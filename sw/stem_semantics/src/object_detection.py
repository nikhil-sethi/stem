import rospy
from transformers import DetrImageProcessor, DetrForObjectDetection
import torch
from PIL import Image
import requests
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class Detection:
    def __init__(self) -> None:
        
        self.device = torch.device('cuda')
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50").to(self.device)
        rospy.init_node("detection_node")
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.detection_callback, queue_size=1)
        self.detection_pub = rospy.Publisher("/object_detections/image/compressed", CompressedImage, queue_size=1)
        # rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage)
        self.detection_timer = rospy.Timer(rospy.Duration(0.05), self.detection_pub_timer)
        self.results = None
        rospy.spin()

    def detection_callback(self, msg):
        self.img_raw = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.img_raw = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=self.img_raw, return_tensors="pt").to(self.device)

        outputs  = self.model(**inputs)
        # convert outputs (bounding boxes and class logits) to COCO API
        # let's only keep detections with score > 0.9
        target_sizes = torch.Tensor([self.img_raw.shape[:2]])
        self.results = self.processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.8)[0]


    def detection_pub_timer(self, event):
        if not self.results:
            return
        img_detections = self.plot_results()
        self.publish_detections(img_detections)

    def plot_results(self):
        img_detect = self.img_raw.copy()
        for score, label, box in zip(self.results["scores"], self.results["labels"], self.results["boxes"]):
            box = [int(round(i, 2)) for i in box.tolist()]
            class_label = self.model.config.id2label[label.item()]
            confidence = round(score.item(), 3)

            x_min, y_min, x_max, y_max = box
            # Draw bounding box
            cv2.rectangle(img_detect, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            # Add label and score
            label_text = f"{class_label}: {confidence:.2f}"
            # Get text size
            (label_width, label_height), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            # Draw rectangle for text background
            cv2.rectangle(img_detect, (x_min, y_min - label_height - 10), (x_min + label_width + 10, y_min), (0, 255, 0), -1)
            # Add text
            cv2.putText(img_detect, label_text, (x_min + 5, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
        return img_detect

    def publish_detections(self, img):
        img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        self.detection_pub.publish(img_msg)

if __name__=="__main__":
    detector = Detection()


