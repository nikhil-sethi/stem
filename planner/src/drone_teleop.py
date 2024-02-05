#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
from std_srvs.srv import Trigger, TriggerResponse, Empty
from planner.base_interface import BaseInterface, BasePlanner
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
        self.pos_tol = 0.1
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


class TeleopInterface(BaseInterface):
    def __init__(self):
        super().__init__("teleop_interface")

        self.planner = KeyboardPlanner(start_pose = self.start_pose)

    def sanity_checks(self):
        self.ready = super().sanity_checks() 

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    teleop_interface = TeleopInterface()
    
    try:
        teleop_interface.run()
    except Exception as e:
        print(e)
    finally:
        pose = PoseStamped()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)