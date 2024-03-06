1#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
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
        rospy.Subscriber('/iris_depth_camera/camera/rgb/image_raw', Image, self.image_callback)

        # Initialize Matplotlib figure and axis
        self.fig, self.ax = plt.subplots()
        res = (480,848)
        self.image = np.zeros(res)
        self.img_plot = self.ax.imshow(self.image)

        # Set up animation
        self.animation = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        plt.show()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def init_plot(self):
        self.img_plot.set_array(self.image)
        return self.img_plot,

    def update_plot(self, frame):
        if self.image is not None:
            # Update the displayed image
            self.img_plot.set_array(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB))
        return self.img_plot,

def main():
    rospy.init_node('image_display_node', anonymous=True)
    image_subscriber = ImageSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
