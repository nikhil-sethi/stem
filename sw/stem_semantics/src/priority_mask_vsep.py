#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from stem_semantics.segmentation_pipelines import ArUcoSegmenter
from stem_semantics.priorities import GTPriorityMap
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
        self.rgb_image_sub = rospy.Subscriber("/rgb_input", Image, self.image_callback)
        self.priority_mask_pub = rospy.Publisher("/mask_output", Image, queue_size=10)
        self.priority_norm_pub = rospy.Publisher("/saliency/image_norm", Image, queue_size=10)
        self.priority_comp_pub = rospy.Publisher("/priority_mask/compressed", CompressedImage, queue_size=10) # for wig calcs

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            header = msg.header
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
            img =img.astype(np.uint8)
            
            seg_msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
            seg_msg.header = header
            seg_msg.header.frame_id = "vi_sensor/camera_left_link" 
            seg_msg.header.stamp = rospy.Time.now()
            self.priority_mask_pub.publish(seg_msg)

            comp_msg = self.bridge.cv2_to_compressed_imgmsg(img)
            comp_msg.header = header
            comp_msg.header.stamp = rospy.Time.now()
            self.priority_comp_pub.publish(comp_msg)

            img_norm = img.copy()
            img_norm[img>0] = 255
            seg_norm_msg = self.bridge.cv2_to_imgmsg(img_norm, encoding="mono8")
            seg_norm_msg.header = header
            seg_norm_msg.header.frame_id = "vi_sensor/camera_left_link" 
            seg_norm_msg.header.stamp = rospy.Time.now()

            self.priority_norm_pub.publish(seg_norm_msg)

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
