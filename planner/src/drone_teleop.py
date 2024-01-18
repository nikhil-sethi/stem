#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
from std_srvs.srv import Trigger, TriggerResponse, Empty

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

class KeyboardController:
    def __init__(self):
        rospy.init_node('keyboard_controller', anonymous=True)

        self.odom_sub = rospy.Subscriber("mavros/local_position/odom", Odometry, callback = self.odom_cb)

        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(50)  # 10 Hz

        self.enable_ext_cont_server_ = rospy.Service("/px4_ext_cont_enable", Trigger, self.start_mission)
        self.autopilot_ctrl = True # flag for when the toolbox is in control
        self.ready = False  # goes from the planner side
        self.mission_finished_client_ = rospy.ServiceProxy("/px4_mission_finished_ext_cont", Trigger);
        self.reset_gazebo_client_ = rospy.ServiceProxy("/gazebo/reset_world", Empty);

        self.start_pos = [0,0,1]
        self.position = [0,0,0]
        self.orientation = [0,0,0]
        self.key = None

    def sanity_checks(self):
        # do some handy checks here to see if the planner is ready
        self.ready = True

    def start_mission(self, req:Trigger):
        
        self.autopilot_ctrl = False
        response = TriggerResponse()
        response.success = self.ready
        if self.ready:
            response.message = "Planner is ready. Sending control commands."
        else:
            response.message = "Planner not ready. Waiting..."
        return response

    def odom_cb(self, msg):
        self.position = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.orientation = euler_from_quaternion(quat)
        # self.velocity = msg.twist.twist.linear_velocity

    def send_pose_command(self, x, y, z, yaw_degrees):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "base_link"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Convert yaw from degrees to quaternion
        yaw_rad = yaw_degrees * (3.141592653589793 / 180.0)
        quaternion = quaternion_from_euler(0, 0, yaw_rad)
        pose_msg.pose.orientation = Quaternion(*quaternion)

        self.pub.publish(pose_msg)

    def run(self):
        x, y, z = self.start_pos
        yaw = 0.0
        tol = 0.2
        while not rospy.is_shutdown():
            self.sanity_checks()
            if self.ready and not self.autopilot_ctrl:
                key = getKey()
                if key == '+':  # thrist increase
                    z += tol
                elif key == '-': # thrust decrease
                    z += -tol
                # NUMPAD ON!
                elif key == '4': # left
                    y += tol
                elif key == '6': # right
                    y += -tol
                elif key == '8': # forward
                    x += tol
                elif key == '2': # backward
                    x += -tol
                elif key == '7':
                    yaw += 10.0 # yaw ccw
                elif key == '9':
                    yaw += -10.0 # yaw cw

                elif key == 'q': # mission end
                    res = self.mission_finished_client_.call()                            
                    self.autopilot_ctrl = res.success
                
                elif key == 'r': # reset gazebo
                    res = self.reset_gazebo_client_.call()                            
                    
                elif key == '\x03': # ctrl-c
                    break

                self.send_pose_command(x, y, z, yaw)
                self.rate.sleep()
        
        rospy.loginfo("Exiting cleanly.")


if __name__ == '__main__':
    controller = KeyboardController()
    settings = termios.tcgetattr(sys.stdin)
    try:
        controller.run()
    except Exception as e:
        print(e)
    finally:
        pose = PoseStamped()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)