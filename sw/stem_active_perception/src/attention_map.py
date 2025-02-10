import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.ndimage import gaussian_filter, sobel
from sensor_msgs.msg import Image, CompressedImage
import rospy
import cv2
from cv_bridge import CvBridge
import time
from std_msgs.msg import Header

class NoName():
    def __init__(self) -> None:
        rospy.init_node("noname")

        res = (480,848)
        self.arr_att = np.zeros(res)
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(self.arr_att, animated=True, origin='upper',cmap='jet', extent=[0, res[1], res[0],0], vmax=1, vmin=0) # the vmax vmin is important so that inital colour range is set. 
        self.bridge = CvBridge()
        rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw/compressed")
        
        cam_sub = rospy.Subscriber(rgb_topic, CompressedImage, self.cam_callback, queue_size=10)
        self.att_pub = rospy.Publisher("/iris_depth_camera/attention_map/compressed", CompressedImage, queue_size=10)

        rospy.wait_for_message(rgb_topic, CompressedImage, timeout=5)
        self.att_header = Header()
        att_timer = rospy.Timer(rospy.Duration(1/15), self.att_map_publisher)
        
        # self.run_animation()
        rospy.spin()

    def att_map_publisher(self, event):
        self.att_msg = self.bridge.cv2_to_compressed_imgmsg(self.arr_att)
        self.att_msg.header = self.att_header
        self.att_pub.publish(self.att_msg)

    def cam_callback(self, msg):
        # arr = msg.data
        start = time.time()
        arr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.arr_att = get_attention_map(arr)
        
        self.att_header = msg.header  # to sync timestamps with RGB message

    def update_ani(self, frame):
        self.im.set_array(self.arr_att)
        # self.im.autoscale()
        return self.im,

    def run_animation(self):
        ani = FuncAnimation(self.fig, self.update_ani, frames=100)
        plt.show()


def reds(img):
    rgb_image /=255
    arr_red = img[:,:,2] + img[:,:,1] - img[:,:,0]
    arr_red = (arr_red + 0.5)/(1.5)
    return arr_red

def blues(img):

    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Define bluish hue range in HSV
    lower_blue = np.array([80, 150, 100])
    upper_blue = np.array([130, 255, 255])

    # Create mask for bluish pixels
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    # Apply mask to HSV image
    blueish_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    # getting a mono images with intensisties only
    arr_blu = np.prod(blueish_image, axis=-1)/(130*255*255) # blacks (low attention) ---> light blues (medium attention) --> high saturation Blue (high attention)

    # Convert HSV back to BGR
    # bgr_blueish_image = cv2.cvtColor(blueish_image, cv2.COLOR_HSV2BGR)

    return mask

def edges(img):
     # geometry (edges, occlusions), happens partially in 3d
    arr_seg = img.copy()
    arr_seg[img<0.0] = 0
    # arr_seg[arr_red>0.5] = 1
    mode = 'nearest'
    sobel_h = sobel(arr_seg, 0, mode=mode)  # horizontal gradient
    sobel_v = sobel(arr_seg, mode = mode)  # vertical gradient
    arr_edges = np.sqrt(sobel_h**2 + sobel_v**2)
    # arr_edges *= 0.2/arr_edges.max()  # normalization
    arr_edges[arr_edges>0]=2
    # arr_edges[arr_edges<0.07] = 0
    return arr_edges

def blur(img):
    # arr_att = gaussian_filter(arr_att, sigma=3)      
    blurred = gaussian_filter(img, sigma=10)    
    return blurred

def saliency(img):
    c = cv2.dft(np.float32(img), flags=cv2.DFT_COMPLEX_OUTPUT)
    mag = np.sqrt(c[:, :, 0] ** 2 + c[:, :, 1] ** 2)
    spectralResidual = np.exp(np.log(mag) - cv2.boxFilter(np.log(mag), -1, (3, 3)))

    c[:, :, 0] = c[:, :, 0] * spectralResidual / mag
    c[:, :, 1] = c[:, :, 1] * spectralResidual / mag
    c = cv2.dft(c, flags=(cv2.DFT_INVERSE | cv2.DFT_SCALE))
    mag = c[:, :, 0] ** 2 + c[:, :, 1] ** 2
    cv2.normalize(cv2.GaussianBlur(mag, (9, 9), 3, 3), mag, 0., 1., cv2.NORM_MINMAX)
    # pyplot.subplot(2, 2, 2)
    # pyplot.imshow(mag)

    return mag


# def gmm(img):
#     gm = GaussianMixture(n_components=5, random_state=0).fit(img)
#     for 



def get_attention_map(rgb_image):
    # top down attention (red patches)
    arr_blu = blues(rgb_image)

    # arr_red = (arr_red + 0.5)/(1.5)

    # arr_blur = gaussian_filter(arr_red, sigma=5)
    # arr_blur = (arr_blur - arr_blur.min())/(arr_blur.max() - arr_blur.min())
    # arr_blur[arr_blur<0.31]=0

    # bottom up saliency 
    # bright, sharp, 

    # print((arr_red>0).shape)
    # geometry (edges, occlusions), happens partially in 3d
    # arr_seg = arr_red.copy()
    # arr_red[arr_red<0.0] = 0
    # arr_red[arr_red>0.0] = 1
    # mode = 'nearest'
    # sobel_h = sobel(arr_seg, 0, mode=mode)  # horizontal gradient
    # sobel_v = sobel(arr_seg, mode = mode)  # vertical gradient
    # arr_edges = np.sqrt(sobel_h**2 + sobel_v**2)
    # # arr_edges *= 0.2/arr_edges.max()  # normalization
    # arr_edges[arr_edges>0]=2
    # # arr_edges[arr_edges<0.07] = 0
    # # print(rgb_image[259,168,1]+rgb_image[259,168,2])
    # # combine attentions
    # arr_att = arr_red + arr_edges
    # arr_att = gaussian_filter(arr_att, sigma=3)      
    # arr_att = gaussian_filter(arr_att, sigma=5)    
    # arr_att[arr_att<0.31]=0
    return arr_blu

# class FakeAttention

if __name__=="__main__":
    # arr = plt.imread("/workspaces/stem_ws/src/thesis/planner/view2.png")
    # arr_att = get_attention_map(arr)
    # plt.imshow(arr_att, cmap='jet')

    # plt.show()

    noname = NoName()
    # noname.run_animation()