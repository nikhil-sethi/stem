#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class IrisVisionStatePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('iris_vision_state_publisher', anonymous=True)
        
        # The name of the model to extract
        self.model_name = "iris_vision"
        
       
        # Wait for the /gazebo/set_model_state service to become available
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

         # Subscribe to the /gazebo/model_states topic
        self.model_states_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.model_states_callback)

        
        rospy.loginfo("IrisVisionStatePublisher initialized and listening to /gazebo/model_states")

    def model_states_callback(self, msg):
        try:
            # Get the index of the model named "iris_vision"
            # index = msg.name.index(self.model_name)

            # Extract the pose and twist of the "iris_vision" model
            iris_vision_state = SetModelStateRequest()
            iris_vision_state.model_state.model_name = self.model_name
            iris_vision_state.model_state.pose = msg.pose.pose
            iris_vision_state.model_state.twist = msg.twist.twist
            iris_vision_state.model_state.reference_frame = "world"

            # Call the set_model_state service
            self.set_model_state_srv(iris_vision_state)
            rospy.loginfo("Called /gazebo/set_model_state service for iris_vision")

        except ValueError:
            rospy.logwarn(f"Model {self.model_name} not found in /gazebo/model_states")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        IrisVisionStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
