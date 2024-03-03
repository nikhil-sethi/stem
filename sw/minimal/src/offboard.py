"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from quadrotor_msgs.msg import PositionCommand
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_multiply, quaternion_from_euler

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def cmd_cb(msg):
    global target_pose

    target_pose = PoseStamped()
    target_pose.pose.position.x = msg.position.x
    target_pose.pose.position.y = msg.position.y
    target_pose.pose.position.z = msg.position.z
    # print(msg.yaw)
    target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,msg.yaw))
    # print(target_pose.pose.orientation)
    # current_cmd = msg

def sensor_pose_cb(msg):
    global sensor_pose, current_odom
    current_odom.header = msg.header
    current_odom.pose.pose = msg.pose.pose

    sensor_pose.header = msg.header
    sensor_pose.header.frame_id = "world"
    q_rot = tf.transformations.quaternion_from_euler(-1.57, 0, -1.57)
    q_orig = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    q_new = quaternion_multiply(q_rot, q_orig)
    sensor_pose.pose = msg.pose.pose
    sensor_pose.pose.orientation = Quaternion(*q_new)
    sensor_pose.pose.position.x += 0.1 

# def tf_publisher(event):
#     pos = 

#     br.sendTransform(self.pos, 
#                               self.quat,
#                               rospy.Time.now(),
#                               f"drone_{j}/odom",
#                               f"world")  



if __name__ == "__main__":
    origin = []
    rospy.init_node("offb_node_py")
    target_pose = PoseStamped()
    sensor_pose = PoseStamped()
    current_odom = Odometry()

    target_pose.pose.position.x = 0
    target_pose.pose.position.y = 0
    target_pose.pose.position.z = 1

    br = tf.TransformBroadcaster()

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    odom_sub = rospy.Subscriber("mavros/odometry/in", Odometry, callback = sensor_pose_cb)
    cmd_sub = rospy.Subscriber("planning/pos_cmd_1", PositionCommand, callback = cmd_cb)

    odom_publisher = rospy.Publisher(f"drone_0/odom", Odometry, queue_size=1)
    sensor_pose_publisher = rospy.Publisher(f"drone_0/camera", PoseStamped, queue_size=1)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(target_pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    # origin = current_odom.pose.pose.position
                last_req = rospy.Time.now()

        local_pos_pub.publish(target_pose)
        sensor_pose_publisher.publish(sensor_pose)
        odom_publisher.publish(current_odom)
        rate.sleep()