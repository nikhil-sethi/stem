"""
Interfacing Racer commands and drone toolbox
"""
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_multiply, quaternion_from_euler
from planner.base_interface import BaseInterface, BasePlanner
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import tf
import tf2_ros

class RacerPlanner(BasePlanner):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.cmd_sub = rospy.Subscriber("planning/pos_cmd_1", PositionCommand, callback = self.cmd_cb)
        self.x, self.y, self.z,  self.yaw  = kwargs["start_pose"]
        self.ready = False

    def cmd_cb(self, msg):
        self.ready = True
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.yaw = msg.yaw*180/3.145
        
    def target_pose(self):
        return self.x, self.y, self.z,  self.yaw 


class RacerInterface(BaseInterface):
    def __init__(self):
        super().__init__("racer_interface")

        self.planner = RacerPlanner(start_pose = self.start_pose)
        self.sensor_pose_publisher = rospy.Publisher(f"drone_0/camera", PoseStamped, queue_size=1)
        self.sensor_pose_timer = rospy.Timer(rospy.Duration(0.01), self.sensor_pose_cb)

        self.cv_bridge = CvBridge()        
        # self.depth_sub = rospy.Subscriber("iris_depth_camera/camera/depth/image_raw", Image, self.depth_mod_cb)
        # self.depth_mod_publisher = rospy.Publisher("drone/img_dep", Image, queue_size=10)
        

    def depth_mod_cb(self, msg):
        "Cut depth map and republish. Racer needs this to detect frontiers"

        depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_image_copy = depth_image.copy()
        

        depth_image_copy[np.isnan(depth_image)] = 0.000
        depth_image_copy[depth_image>=3] = 0.000
        # print(depth_image_copy[400,100])
        modified_depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_image_copy, encoding="passthrough")
        modified_depth_msg.header = msg.header

        self.depth_mod_publisher.publish(modified_depth_msg)

    def sensor_pose_cb(self, event):
        sensor_pose = PoseStamped()
        sensor_pose.header.stamp = rospy.Time.now()
        sensor_pose.header.frame_id = "world"
        try:
            self.t.waitForTransform("world", "drone_0/camera", rospy.Time(), rospy.Duration(8.0))
            pos, quat = self.t.lookupTransform("world", "drone_0/camera", rospy.Time())

            sensor_pose.pose.position = Point(*pos)
            sensor_pose.pose.position.z -= 0.2267 # compensation for the hovergames thing
            sensor_pose.pose.orientation = Quaternion(*quat)
            self.sensor_pose_publisher.publish(sensor_pose)
        except tf2_ros.TransformException:
            print("Tf returned a tranform error. Trying again..")
            pass

    def sanity_checks(self):
        self.ready = super().sanity_checks()*self.planner.ready

if __name__ == "__main__":
    racer_interface = RacerInterface()
    
    try:
        racer_interface.run()
    except Exception as e:
        print(e)