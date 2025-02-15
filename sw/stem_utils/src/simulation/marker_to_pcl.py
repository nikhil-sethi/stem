#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class MarkerToPointCloud2:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('marker_to_pointcloud2')

        # Subscribe to the visualization_msgs/Marker topic
        self.marker_sub = rospy.Subscriber('/planning/travel_traj', Marker, self.marker_callback)

        # Publisher for the sensor_msgs/PointCloud2 topic
        self.pointcloud2_pub = rospy.Publisher('/pointcloud2_topic', PointCloud2, queue_size=10)

    def marker_callback(self, marker_msg):
        # Prepare the header for PointCloud2 message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = marker_msg.header.frame_id

        # Prepare the fields for PointCloud2
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1)
        ]

        # Create a list of points
        points = [(point.x, point.y, point.z, point.z) for point in marker_msg.points]

        # Create the PointCloud2 message
        pointcloud2_msg = pc2.create_cloud(header, fields, points)

        # Publish the PointCloud2 message
        self.pointcloud2_pub.publish(pointcloud2_msg)

if __name__ == '__main__':
    try:
        # Create the MarkerToPointCloud2 object and start the node
        marker_to_pointcloud2 = MarkerToPointCloud2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
