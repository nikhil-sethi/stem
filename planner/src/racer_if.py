"""
Interfacing Racer commands and drone toolbox
"""
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from quadrotor_msgs.msg import PositionCommand
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_multiply, quaternion_from_euler
from planner.base_interface import BaseInterface, BasePlanner
from std_msgs.msg import Header


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
        self.sensor_pose_timer = rospy.Timer(rospy.Duration(0.05), self.sensor_pose_cb)

    def sensor_pose_cb(self, event):
        sensor_pose = PoseStamped()
        sensor_pose.header = self.pose_header
        sensor_pose.header.frame_id = "world"
        q_rot = tf.transformations.quaternion_from_euler(-1.57, 0, -1.57)
        # quaternion_z = quaternion_from_euler(0, 0, -1.57, 'rxyz')
        # quaternion_y = quaternion_from_euler(0, 1.57, 0, 'rxyz')
        # q_rot = quaternion_multiply(quaternion_y, quaternion_z)

        # q_rot = [0.72403681 0.44699833 0.44699833 0.27596319]
        
        q_orig = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        q_new = quaternion_multiply(q_rot, q_orig)

        sensor_pose.pose.position = self.position
        sensor_pose.pose.position.x += 0.1 
        
        sensor_pose.pose.orientation = Quaternion(*q_new)
        
        self.sensor_pose_publisher.publish(sensor_pose)


    def sanity_checks(self):
        self.ready = super().sanity_checks()*self.planner.ready

if __name__ == "__main__":
    racer_interface = RacerInterface()
    
    try:
        racer_interface.run()
    except Exception as e:
        print(e)