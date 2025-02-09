import cv2
import numpy as np

class BaseSegmenter:
    def segment(self, cv_image):
        raise NotImplementedError

class ArUcoSegmenter(BaseSegmenter):
    def __init__(self, aruco_dict = cv2.aruco.DICT_ARUCO_ORIGINAL):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        
        # Detect ArUco markers
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)

    def segment(self, cv_image):
        # Convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        
        # Draw markers on the segmented image
        segmented_image = np.zeros_like(gray)
        
        # print(corners)
        if ids is not None:
            ids[ids>23] = 1 # remove all objects out of set. 1 will turn to 0

            # segmented_image = cv2.aruco.drawDetectedMarkers(segmented_image, corners)
            for i in range(len(ids)):
                corner = np.int0(corners[i]).reshape(-1, 2)
                cv2.fillPoly(segmented_image, [corner], int(ids[i])) 

        return segmented_image

class GazeboSegmenter:
    pass