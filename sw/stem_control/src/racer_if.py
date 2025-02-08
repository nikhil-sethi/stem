"""
Interfacing Racer commands and drone toolbox
"""
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_multiply, quaternion_from_euler
from stem_control.base_interface import BaseInterface, BasePlanner
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
from std_srvs.srv import Trigger
import sys, select, termios, tty


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardPlanner(BasePlanner):
    def __init__(self, *args, **kwargs):
        self.pos_tol = 0.2
        self.x, self.y, self.z,  self.yaw  = kwargs["start_pose"]
        super().__init__()

    def target_pose(self):
        key = getKey()
        # print(key)
        if key == '+':  # thrist increase
            self.z += self.pos_tol
        elif key == '-': # thrust decrease
            self.z += -self.pos_tol
        # NUMPAD ON!
        elif key == '4': # left
            self.y += self.pos_tol
        elif key == '6': # right
            self.y += -self.pos_tol
        elif key == '8': # forward
            self.x += self.pos_tol
        elif key == '2': # backward
            self.x += -self.pos_tol
        elif key == '7':
            self.yaw += 10.0 # yaw ccw
        elif key == '9':
            self.yaw += -10.0 # yaw cw

        elif key == 'q': # mission end
            self.end_mission = True

        elif key == 'r': # reset gazebo
            res = self.reset_gazebo_client_.call()                            
            
        elif key == '\x03': # ctrl-c
            self.end_mission = True

        return self.x, self.y, self.z, self.yaw


class RacerPlanner(BasePlanner):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.cmd_sub = rospy.Subscriber("planning/pos_cmd", PositionCommand, callback = self.cmd_cb)
        self.x, self.y, self.z,  self.yaw  = kwargs["start_pose"]
        self.ready = False 

    def cmd_cb(self, msg):
        # self.ready = True
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
        # self.planner = KeyboardPlanner(start_pose = self.start_pose)  # enable this if you want all planning to happen but use control yourself
        self.sensor_pose_publisher = rospy.Publisher(f"camera/pose", PoseStamped, queue_size=1)
        self.sensor_pose_timer = rospy.Timer(rospy.Duration(0.05), self.sensor_pose_cb)

        self.trigger = rospy.Publisher(f"/move_base_simple/goal", PoseStamped, queue_size=1)

        self.cv_bridge = CvBridge()        
        # self.depth_sub = rospy.Subscriber("iris_depth_camera/camera/depth/image_raw", Image, self.depth_mod_cb)
        # self.depth_mod_publisher = rospy.Publisher("drone/img_dep", Image, queue_size=10)
        
    def start_mission(self, req:Trigger):
        res = super().start_mission(req)

        msg = PoseStamped()
        self.trigger.publish(msg)
        self.planner.ready = True

        return res

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

        # correctly set stamp so that message filter sync works with racer
        if self.sim:
            stamp = rospy.Time.now() # because gazebo depth cam starts from 0
        else:
            stamp = rospy.Time.from_sec(time.time()) # because realsense depth cam uses unix time
        sensor_pose.header.stamp = stamp
        
        try:
            sensor_pose.header.frame_id = "map"
            self.t.waitForTransform("map", "camera_link", rospy.Time(), rospy.Duration(4.0))
            pos, quat = self.t.lookupTransform("map", "camera_link", rospy.Time())

            sensor_pose.pose.position = Point(*pos)
            # sensor_pose.pose.position.z -= 0.2267 # compensation for the hovergames thing
            sensor_pose.pose.orientation = Quaternion(*quat)
            self.sensor_pose_publisher.publish(sensor_pose)
        except:
            pass
    
    def sanity_checks(self):
        self.ready = super().sanity_checks()*self.planner.ready

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    racer_interface = RacerInterface()
    
    try:
        racer_interface.run()
    except Exception as e:
        print(e)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)