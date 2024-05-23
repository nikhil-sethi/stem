from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import pandas as pd
import numpy as np
import rospy
import time

bridge = CvBridge()


def detection_callback(msg):
    img = bridge.compressed_imgmsg_to_cv2(msg)
    areas = [(img == i*255/5).sum()/arr_max for i in range(1,6)]
    time_elapsed = time.time() - start 
    areas.append(time_elapsed)
    if (int(time_elapsed*10)%5==0):
        data.append(areas)


if __name__ == "__main__":
    rospy.init_node("metrics_node")
    arr_max = 480*848
    start = time.time()
    data = []

    sub = rospy.Subscriber("/attention_map/2d/compressed", CompressedImage, detection_callback)
    
    rospy.spin()

    np.savetxt("/root/thesis_ws/src/thesis/sw/metrics/src/areas.csv", data, delimiter=",", fmt="%.5f")