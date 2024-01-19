import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Pose, Quaternion
from std_srvs.srv import Trigger, TriggerResponse, Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time

class BasePlanner:
    def __init__(self, *args, **kwargs) -> None:
        # self.active = True
        self.end_mission = False

    def target_pose(self) -> tuple:
        """
        This function returns a target pose. Needs to be overriden.

        Return: x, y, z, yaw
        """
        raise NotImplementedError


class BaseInterface:
    """A base class to interface with drone toolbox and your 
    custom Planner/controller implementation"""

    def __init__(self, node_name="base_interface") -> None:
        rospy.init_node(node_name, anonymous=True)

        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, callback = self.odom_callback)
        self.pose = PoseStamped()
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(50)  # 10 Hz

        self.enable_ext_cont_server_ = rospy.Service("/px4_ext_cont_enable", Trigger, self.start_mission)
        self.autopilot_ctrl = True # flag for when the toolbox is in control
        self.ready = False  # goes from the planner side
        self.mission_finished_client_ = rospy.ServiceProxy("/px4_mission_finished_ext_cont", Trigger);
        self.reset_gazebo_client_ = rospy.ServiceProxy("/gazebo/reset_world", Empty);

        self.planner = BasePlanner()
        self.start_pose = [0, 0, 0.5, 0]

        rospy.sleep(0.5) # for callbacks to get updated before things get inherited

    def sanity_checks(self):
        if self.planner.end_mission:
            res = self.mission_finished_client_.call()                            
            self.autopilot_ctrl = res.success

        # do some handy checks here to see if the planner is ready
        self.ready = (not self.planner.end_mission)*(not self.autopilot_ctrl)

        return self.ready

    def start_mission(self, req:Trigger):
        
        self.autopilot_ctrl = False
        response = TriggerResponse()
        response.success = self.ready
        if self.ready:
            response.message = "Planner is ready. Sending control commands."
        else:
            response.message = "Planner not ready. Waiting..."
        return response

    def end_mission(self):
        res = self.mission_finished_client_.call()                            
        self.autopilot_ctrl = res.success

    def odom_callback(self, msg):
        # just for easy accesses
        # self.pose = msg.pose
        self.pose_header = msg.header
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        # self.vel
        # self.position = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        # quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        # self.orientation = euler_from_quaternion(quat)
        # self.velocity = msg.twist.twist.linear_velocity

    def send_pose_command(self, x, y, z, yaw_degrees):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "base_link"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Convert yaw from degrees to quaternion
        yaw_rad = yaw_degrees * (3.141592653589793 / 180.0)
        quaternion = quaternion_from_euler(0, 0, yaw_rad)
        pose_msg.pose.orientation = Quaternion(*quaternion)

        self.pose_pub.publish(pose_msg)

    def run(self):
        try:
            while not rospy.is_shutdown() and not self.planner.end_mission:
                self.sanity_checks()
                if self.ready:
                    x, y, z, yaw = self.planner.target_pose()
                    print(x, y, z, yaw)
                else:
                    x, y, z, yaw = self.start_pose
                
                self.send_pose_command(x, y, z, yaw)

                self.rate.sleep()
            self.mission_finished_client_.call()
        except KeyboardInterrupt:
            self.mission_finished_client_.call()
        finally:
            raise
        rospy.loginfo("Exiting cleanly.")



if __name__=="__main__":
    base_interface = BaseInterface()
    base_interface.run()
