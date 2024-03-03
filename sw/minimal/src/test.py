import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
from tf.transformations import quaternion_multiply, quaternion_from_euler
import tf
import time


def base_pose_timer(event):
    base_pose = PoseStamped()
    base_pose.header.stamp = rospy.Time.from_sec(time.time())
    base_pose.header.frame_id = "map"
    base_pose.pose.position = Point(1,0,0)
    quat = quaternion_from_euler(0,0,0)
    base_pose.pose.orientation = Quaternion(*quat)
    pose_pub.publish(base_pose)

def sensor_pose_cb(msg):
    sensor_pose = PoseWithCovarianceStamped()
    sensor_pose.header.stamp = rospy.Time.from_sec(time.time())
    sensor_pose.header.frame_id = "map"
    # sensor_pose.pose.pose.position = msg.pose.position
    # sensor_pose.pose.pose.orientation = msg.pose.orientation
    quat = quaternion_from_euler(0,0,0)
    sensor_pose.pose.pose.position = Point(1,0,0)
    sensor_pose.pose.pose.orientation = Quaternion(*quat)

    # sensor_pose.header.frame_id = "world"
    # pos, quat = t.lookupTransform("world", "camera_link", rospy.Time())

    # sensor_pose.pose.position = Point(*pos)
    # sensor_pose.pose.orientation = Quaternion(*quat)

    pose_cov_publisher.publish(sensor_pose)

rospy.init_node("test")
pose_cov_publisher = rospy.Publisher(f"/pose_cov", PoseWithCovarianceStamped, queue_size=1)
pose_pub = rospy.Publisher(f"/dummy_pose", PoseStamped, queue_size=1)
pose_timer = rospy.Timer(rospy.Duration(0.1), base_pose_timer)

pose_sub = rospy.Subscriber("/pose_topic", PoseStamped, callback = sensor_pose_cb)
# br = tf.TransformBroadcaster()
# t = tf.TransformListener()
# sensor_pose_publisher = rospy.Publisher(f"mavros/vision_pose/pose", PoseStamped, queue_size=1)
# sensor_pose_timer = rospy.Timer(rospy.Duration(0.1), sensor_pose_cb)
rospy.spin()