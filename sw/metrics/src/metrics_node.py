from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import rospy
import time
import message_filters
from common_msgs.msg import float64List
from common_msgs.msg import Float64Stamped
from semantics.segmentation_pipelines import ArUcoSegmenter
from semantics.priorities import create_label_map
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

class Metrics:
    def __init__(self) -> None:
        # Params
        self.data = {"timestamp":[], "path_length":[], "time":[], "energy":[], "wif":[0]}
        self.start_time = time.time()
        self.num_pixels = 480*848
        self.bridge = CvBridge()
        self.segmenter = ArUcoSegmenter()
        # self.relevant_classes = ["human", "plant", "table", "carpet", "dog", "wall", "blood", "rubble", "flashlight"] # only do segmentation area calc for these classes. just for compute saving
        self.relevant_classes = ["human", "flashlight", "blood", "rope", "dog", "radio"] # only do segmentation area calc for these classes. just for compute saving
        self.label_map = create_label_map("/root/thesis_ws/src/thesis/sw/perception/semantics/src/labels.txt") # classes -> ids
        print(self.label_map)
        # initialise area lists
        for cls in self.relevant_classes:
            self.data[cls] = []

        # initialise to 
        self.weighted_entropy_old = rospy.wait_for_message("/data/weighted_entropy", Float64Stamped).data
        
        # ROS
        self.img_sub = message_filters.Subscriber("/camera/color/image_raw/compressed", CompressedImage)
        self.traj_sub =  message_filters.Subscriber("/data/trajectory_info", float64List)
        self.we_sub = message_filters.Subscriber("/data/weighted_entropy", Float64Stamped)
        

        ts = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.traj_sub, self.we_sub], 10, 0.1)
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
            self.data[cls].append(cls_area)

        # weighted information gain
    
        info_gain = self.weighted_entropy_old - weighted_entropy_msg.data

        self.data["wif"].append(self.data["wif"][-1] + info_gain) # cumulative
        self.weighted_entropy_old =  weighted_entropy_msg.data
        # print(self.data)

    def finish_metrics_cb(self, req):
        """Post process and compile data"""
        
        # save data
        self.data["wif"] = self.data["wif"][1:]
        df = pd.DataFrame(data=self.data)
        df.to_csv(f"/root/thesis_ws/src/thesis/sw/metrics/src/{time.ctime()}.csv", float_format="%.5f")
        
        # exit(0)
        rospy.signal_shutdown("Sd")

        # plot results
        # self.plot_results(df)

    # def plot_results(self, df):
    #     fig1, ax1 = plt.subplots()
    #     # Plot information gain
    #     color = 'tab:blue'
    #     ax1.set_xlabel('Time')
    #     ax1.set_ylabel('Entropy')
    #     times = df["timestamp"]
    #     ax1.plot(times, df["wif"], color=color)

    #     # find target timestamp
    #     success_idx = (df["timestamp"][df["human"]]>=0.02)[0]  # first instance when target area crosses threshold
    #     success_times = df["timestamp"][success_idx.index] - df["timestamp"]
    #     success_path_length = df["path_length"]

if __name__ == "__main__":
    rospy.init_node("metrics_node")
    
    metrics_node = Metrics()    
    rospy.spin()

