1#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None

        # Create a subscriber for the image topic
        topic = rospy.get_param("~img_topic", "/camera/depth/image_rect_raw/compressed")
        rospy.Subscriber(topic, CompressedImage, self.image_callback)

        # Initialize Matplotlib figure and axis
        self.fig, self.ax = plt.subplots()
        res = (480,848)
        self.image = np.zeros(res)
        self.img_plot = self.ax.imshow(self.image, vmax=5, vmin=0)

        # Set up animation
        self.animation = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        plt.show()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def init_plot(self):
        self.img_plot.set_array(self.image)
        return self.img_plot,

    def update_plot(self, frame):
        if self.image is not None:
            # Update the displayed image
            # self.img_plot.set_array(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB))
            self.img_plot.set_array(self.image)
            # print(np.unique(self.image))
            # self.img_plot.set_array(self.image)
        return self.img_plot,

def main():
    rospy.init_node('image_display_node', anonymous=True)
    image_subscriber = ImageSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
