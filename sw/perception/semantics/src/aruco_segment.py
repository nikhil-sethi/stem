#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from semantics.segmentation_pipelines import ArUcoSegmenter
from semantics.priorities import GTPriorityMap
import numpy as np

class PriorityMask:
    def __init__(self) -> None:
        rospy.init_node('aruco_segmenter', anonymous=True)

        # semantic segmentation pipeline (color -> labels)
        self.segmenter = ArUcoSegmenter()

        # priority inference
        # self.priority_map = SemanticPriorityMap()
        self.priority_map = GTPriorityMap("human", context="mine", p_max=8)

        # ROS
        self.bridge = CvBridge()
        self.rgb_image_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback)
        self.priority_mask_pub = rospy.Publisher("/priority_mask/compressed", CompressedImage, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            header = msg.header
            cv_image_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        seg_img = self.segmenter.segment(cv_image_bgr)
        # print(seg_img.max())
        priority_mask_img = self.priority_map.priorities[seg_img] # pixelwise integer labels -> integer priorities
        # print(np.unique(priority_mask_img))
        self.publish_img(priority_mask_img*255/self.priority_map.p_max, header)


    def publish_img(self, img, header):
         # Publish the segmented image
        try:
            seg_msg = self.bridge.cv2_to_compressed_imgmsg(img)
            seg_msg.header = header
            seg_msg.header.stamp = rospy.Time.now()
            self.priority_mask_pub.publish(seg_msg)
        except CvBridgeError as e:
            rospy.logerr(e)


def main():
    priority_mask = PriorityMask()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
