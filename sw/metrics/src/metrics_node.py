from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import pandas as pd
import rospy
import time
import message_filters
from stem_msgs.msg import float64List
from stem_msgs.msg import Float64Stamped
from semantics.segmentation_pipelines import ArUcoSegmenter
from semantics.priorities import create_label_map
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import math


def norm(point_a, point_b):
    return  math.sqrt((point_a.x - point_b.x)**2+ (point_a.y - point_b.y)**2 + (point_a.z - point_b.z)**2)


class Metrics:
    def __init__(self) -> None:
        # Params
        world = rospy.get_param("/world", "earthquake")

        self.data = {"timestamp":[], "path_length":[], "time":[], "energy":[], "wif":[0]}
        self.start_time = time.time()
        self.num_pixels = 480*848
        self.bridge = CvBridge()
        self.segmenter = ArUcoSegmenter()
        if world =="earthquake":
            self.relevant_classes = ["human", "plant", "table", "carpet", "dog", "wall", "blood", "rubble", "flashlight"] # only do segmentation area calc for these classes. just for compute saving
        else:
            self.relevant_classes = ["human", "flashlight", "blood", "rope", "dog", "radio"] # only do segmentation area calc for these classes. just for compute saving
        
        self.label_map = create_label_map("/root/thesis_ws/src/thesis/sw/perception/semantics/src/labels.txt") # classes -> ids  TODO make path relative
        print(self.label_map)

        self.is_vsep_comparison = False # make true when comparing to VSEP

        # initialise area lists
        for cls in self.relevant_classes:
            self.data[cls] = []

        # initialise to 
        self.weighted_entropy_old = rospy.wait_for_message("/data/weighted_entropy", Float64Stamped, timeout=30).data
        
        # ------ FOR VSEP. REMOVE OTHERWISE
        if self.is_vsep_comparison:
            # metrics
            self.path_length = 0
            self.elapsed_time = 0

            # ROS stuffs
            msg = rospy.wait_for_message("/firefly/odometry_sensor1/odometry", Odometry,timeout=20)
            self.last_pos = msg.pose.pose.position
            self.start_time = msg.header.stamp
            self.last_time = self.start_time
            # ------ FOR VSEP

        # ROS
        self.img_sub = message_filters.Subscriber("/camera/color/image_raw/compressed", CompressedImage)
        
        if self.is_vsep_comparison:
            self.traj_sub =  message_filters.Subscriber("/firefly/odometry_sensor1/odometry", Odometry) # NEED FOR VSEP COMPARISON
        else:
            self.traj_sub =  message_filters.Subscriber("/data/trajectory_info", float64List)
        self.we_sub = message_filters.Subscriber("/data/weighted_entropy", Float64Stamped)
        

        ts = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.traj_sub, self.we_sub], 10, 0.1)
        
        if self.is_vsep_comparison:
            ts.registerCallback(self.vsep_data_callback) # VSEP COMPARISON
        else:
            ts.registerCallback(self.data_callback)
    

        self.finish_metrics_service = rospy.Service("/finish_metrics_service", Trigger, self.finish_metrics_cb)

    def data_callback(self, img_msg:CompressedImage, traj_info_msg, weighted_entropy_msg:Float64Stamped):
        time_now = rospy.Time.now().to_sec()

        # # throttle calcs at frequency
        # if (int(time_now-self.start_time*10)%5==0):
        #     return

        # timestamp
        self.data["timestamp"].append(time_now)

        # Trajectory
        self.data["path_length"].append(traj_info_msg.data[0])
        self.data["time"].append(traj_info_msg.data[1])
        self.data["energy"].append(traj_info_msg.data[2])

        # Segmentation area for each relevant class 
        img = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        seg_img = self.segmenter.segment(img)

        for cls in self.relevant_classes:
            label = self.label_map[cls]+1
            # segmentation area
            cls_area = (seg_img == label).sum()/self.num_pixels 
            if cls=="human" and cls_area>0.02:
                print(f"Target found! time: {traj_info_msg.data[1]}, path_length:{traj_info_msg.data[0]}")
                rospy.signal_shutdown("Target found")
            self.data[cls].append(cls_area)

        # weighted information gain
    
        info_gain = self.weighted_entropy_old - weighted_entropy_msg.data

        self.data["wif"].append(self.data["wif"][-1] + info_gain) # cumulative
        self.weighted_entropy_old =  weighted_entropy_msg.data
        # print(self.data)

    def vsep_data_callback(self, img_msg:CompressedImage, odom_msg:Odometry, weighted_entropy_msg:Float64Stamped):
        # timestamp
        self.elapsed_time = (odom_msg.header.stamp - self.start_time).to_sec()
        # self.last_time = odom_msg.header.stamp

        # path length
        pos = odom_msg.pose.pose.position   # extract current pos from msg

        diff = norm(pos, self.last_pos)
        self.path_length += diff
        self.last_pos = pos


        traj_info_msg  = float64List()
        traj_info_msg.data.append(self.path_length)
        traj_info_msg.data.append(self.elapsed_time)
        traj_info_msg.data.append(0) # don't compute energy because there is no trajectory

        self.data_callback(img_msg, traj_info_msg, weighted_entropy_msg)

    def finish_metrics_cb(self, req):
        """Post process and compile data"""
        
        # save data
        self.data["wif"] = self.data["wif"][1:]
        df = pd.DataFrame(data=self.data)
        df.to_csv(f"/root/thesis_ws/src/thesis/sw/metrics/src/{time.ctime()}.csv", float_format="%.5f") # TODO make relative
        
        rospy.signal_shutdown("Finish metrics service called. Closing everything")


if __name__ == "__main__":
    rospy.init_node("metrics_node")
    
    metrics_node = Metrics()    
    rospy.spin()

