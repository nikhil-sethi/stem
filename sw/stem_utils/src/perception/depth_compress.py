#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs


class DepthImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('depth_image_subscriber', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw/compressed', CompressedImage, self.image_callback)
        # self.image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, data):
        try:
            # Convert the compressed image to a numpy array
            # np_arr = np.fromstring(data.data, np.uint8)
            # Decode the image
            # depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            
            depth_image = self.bridge.compressed_imgmsg_to_cv2(data)
            # Process the depth image
            self.process_depth_image(depth_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def process_depth_image(self, depth_image):
        # Example processing: print the depth value at the center of the image
        height, width = depth_image.shape
        center_depth_value = depth_image[height // 2, width // 2]
        rospy.loginfo(f"Depth value at center: {center_depth_value}")

class DepthImageFromBag:
    def __init__(self):
        cfg = rs.config()
        cfg.enable_device_from_file("/root/outdoors.bag");
        pipeline = rs.pipeline()
        profile = pipeline.start(cfg)

        try:
            while True:
                # Create a pipeline object. This object configures the streaming camera and owns it's handle
                frames = pipeline.wait_for_frames()
                depth = frames.get_depth_frame()
                if not depth: continue

                # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
                coverage = [0]*64
                # for y in range(480):
                #     for x in range(640):
                dist = depth.get_distance(282, 295)
                print(dist)
                        # if 0 < dist and dist < 1:
                        #     coverage[x//10] += 1

                    # if y%20 is 19:
                    #     line = ""
                    #     for c in coverage:
                    #         line += " .:nhBXWW"[c//25]
                    #     coverage = [0]*64
                    #     print(line)

        finally:
            pipeline.stop()
            
if __name__ == '__main__':
    try:
        DepthImageFromBag()
    except rospy.ROSInterruptException:
        pass
