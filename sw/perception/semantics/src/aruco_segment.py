#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ArUcoSegmenter:
    def __init__(self):
        rospy.init_node('aruco_segmenter', anonymous=True)
        self.bridge = CvBridge()

        # Subscribe to the RGB image topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback)

        # Publisher for the segmented image
        self.segmented_image_pub = rospy.Publisher("/attention_map/2d/compressed", CompressedImage, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            header = msg.header
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        # Draw markers on the segmented image
        segmented_image = np.zeros_like(gray)
        
        # print(corners)
        if ids is not None:
            # segmented_image = cv2.aruco.drawDetectedMarkers(segmented_image, corners)
            for i in range(len(ids)):
                corner = np.int0(corners[i]).reshape(-1, 2)
                cv2.fillPoly(segmented_image, [corner], (int((ids[i]+1)*255/5))) 
        

        # Publish the segmented image
        try:
            seg_msg = self.bridge.cv2_to_compressed_imgmsg(segmented_image)
            seg_msg.header = header
            seg_msg.header.stamp = rospy.Time.now()
            self.segmented_image_pub.publish(seg_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    aruco_segmenter = ArUcoSegmenter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
