#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import os

class MeshPublisherNode:
    def __init__(self):
        rospy.init_node('mesh_publisher_node', anonymous=True)
        
        self.mesh_file_path = "/workspaces/stem_ws/src/thesis/sw/stem_bringup/models/person_standing/meshes/standing.dae"
        if not self.mesh_file_path:
            rospy.logerr("No mesh file path provided!")
            return
        
        self.marker_pub = rospy.Publisher('mesh_marker', Marker, queue_size=10)
        
        self.mesh_marker = Marker()
        self.mesh_marker.header.frame_id = "map"
        self.mesh_marker.type = Marker.MESH_RESOURCE
        self.mesh_marker.action = Marker.ADD
        self.mesh_marker.scale.x = 1
        self.mesh_marker.scale.y = 1
        self.mesh_marker.scale.z = 1
        # self.mesh_marker.color.a = 1.0
        # self.mesh_marker.color.r = 0.0
        # self.mesh_marker.color.g = 1.0
        # self.mesh_marker.color.b = 0.0
        self.mesh_marker.pose.orientation.w = 1.0
        self.mesh_marker.pose.position.x = 0
        self.mesh_marker.pose.position.y = 0
        self.mesh_marker.pose.position.z = 0
        self.mesh_marker.mesh_resource = "file://" + os.path.abspath(self.mesh_file_path)
        print(self.mesh_marker.mesh_resource)
        self.mesh_marker.mesh_use_embedded_materials = True

    def publish_mesh(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.mesh_marker.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.mesh_marker)
            rate.sleep()

if __name__ == '__main__':
    try:
        mesh_publisher = MeshPublisherNode()
        mesh_publisher.publish_mesh()
    except rospy.ROSInterruptException:
        pass
