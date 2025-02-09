#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from stem_semantics.segmentation_pipelines import ArUcoSegmenter
from stem_semantics.priorities import GTPriorityMap
import numpy as np
import cv2
import time

class PriorityMask:
    def __init__(self) -> None:
        rospy.init_node('aruco_segmenter', anonymous=True)
        world = rospy.get_param("/world", "earthquake")

        # semantic segmentation pipeline (color -> labels)
        self.segmenter = ArUcoSegmenter()

        # priority inference
        # self.priority_map = SemanticPriorityMap()
        self.priority_map = GTPriorityMap("human", context=world, p_max=8)
        print(self.priority_map.priorities)
        # ROS
        self.bridge = CvBridge()
        self.rgb_image_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback)
        self.priority_mask_pub = rospy.Publisher("/priority_mask/compressed", CompressedImage, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            header = msg.header
            cv_image_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        seg_img = self.segmenter.segment(cv_image_bgr)
        priority_mask_img = self.priority_map.priorities[seg_img] # pixelwise integer labels -> integer priorities
        # start = time.perf_counter()
        # priority_mask_img = np.zeros(cv_image_bgr.shape[:-1])

        # cv_image_bgr = cv2.cvtColor(cv_image_bgr, cv2.COLOR_RGB2HSV_FULL)
        # cv_image_bgr[cv_image_bgr[:,:,2]<40]=0
        # cv_image_bgr[cv_image_bgr[:,:,1]>250]=0

        # hue_channel = cv_image_bgr[:,:,0]
        # priority_mask_img[hue_channel>160] = 8
        # priority_mask_img[(hue_channel>150) * (hue_channel<160)] = 7
        # priority_mask_img[(hue_channel>60) * (hue_channel<100)] = 5
        # priority_mask_img[(hue_channel>30) * (hue_channel<60)] = 3
        # print(time.perf_counter()-start)
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
