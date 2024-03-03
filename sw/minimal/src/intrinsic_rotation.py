import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def rotate_pose_by_xyz(pose_stamped, angles):
    # Create a TransformListener
    listener = tf.TransformListener()

    # Wait for the transformation to be available
    listener.waitForTransform(pose_stamped.header.frame_id, "rotated_frame", rospy.Time(), rospy.Duration(4.0))

    # Convert PoseStamped to a Pose object
    pose = pose_stamped.pose

    # Create a quaternion from the original pose
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # Apply intrinsic XYZ rotation to the quaternion
    rotated_quaternion = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2], 'rxyz')
    rotated_quaternion = tf.transformations.quaternion_multiply(rotated_quaternion, quaternion)

    # Create a new PoseStamped with the rotated orientation
    rotated_pose_stamped = PoseStamped()
    rotated_pose_stamp
    
    ed.header = Header()
    rotated_pose_stamped.header.frame_id = pose_stamped.header.frame_id
    rotated_pose_stamped.header.stamp = rospy.Time.now()

    rotated_pose_stamped.pose.position = pose.position
    rotated_pose_stamped.pose.orientation.x = rotated_quaternion[0]
    rotated_pose_stamped.pose.orientation.y = rotated_quaternion[1]
    rotated_pose_stamped.pose.orientation.z = rotated_quaternion[2]
    rotated_pose_stamped.pose.orientation.w = rotated_quaternion[3]

    # Transform the rotated pose to the original frame
    rotated_pose_stamped_transformed = listener.transformPose(pose_stamped.header.frame_id, rotated_pose_stamped)

    return rotated_pose_stamped_transformed

if __name__ == "__main__":
    rospy.init_node("pose_rotator")

    # Assuming you have a subscriber for the original pose and a publisher for the rotated pose
    original_pose_subscriber = rospy.Subscriber("/original_pose_topic", PoseStamped, pose_callback)
    rotated_pose_publisher = rospy.Publisher("/rotated_pose_topic", PoseStamped, queue_size=10)

    rospy.spin()