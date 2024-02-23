import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.ndimage import gaussian_filter, sobel
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge

class NoName():
    def __init__(self) -> None:
        rospy.init_node("noname")

        res = (480,848)
        self.arr_att = np.zeros(res)
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(self.arr_att, animated=True, origin='upper',cmap='jet', extent=[0, res[1], res[0],0], vmax=1, vmin=0) # the vmax vmin is important so that inital colour range is set. 
        self.bridge = CvBridge()

        cam_sub = rospy.Subscriber("/mantis/camera/rgb/image_raw", Image, self.cam_callback, queue_size=10)
        self.att_pub = rospy.Publisher("attention_map/2d", Image, queue_size=10)

        # att_timer = rospy.Timer(rospy.Duration(0.1), self.att_map_publisher)
        
        # self.run_animation()
        rospy.spin()

    def att_map_publisher(self, event):
        ros_image = self.bridge.cv2_to_imgmsg(self.arr_att)
        ros_image.header.frame_id = "world"
        self.att_pub.publish(ros_image)

    def cam_callback(self, msg):
        # arr = msg.data
        arr = self.bridge.imgmsg_to_cv2(msg).astype(float)
        self.arr_att = get_attention_map(arr)

        att_msg = self.bridge.cv2_to_imgmsg((self.arr_att*230).astype(np.uint8), encoding='mono8')
        att_msg.header = msg.header
        # print(att_msg.)
        self.att_pub.publish(att_msg)


    def update_ani(self, frame):
        self.im.set_array(self.arr_att)
        # self.im.autoscale()
        return self.im,

    def run_animation(self):
        ani = FuncAnimation(self.fig, self.update_ani, frames=100)
        plt.show()

def get_attention_map(rgb_image):
    # top down attention (red patches)
    rgb_image /=255
    arr_red = rgb_image[:,:,0]-(rgb_image[:,:,1] + rgb_image[:,:,2])
    arr_red = (arr_red + 0.5)/(1.5)

    arr_blur = gaussian_filter(arr_red, sigma=5)
    # arr_blur = (arr_blur - arr_blur.min())/(arr_blur.max() - arr_blur.min())
    arr_blur[arr_blur<0.31]=0

    # bottom up saliency 
    # bright, sharp, 

    # print((arr_red>0).shape)
    # geometry (edges, occlusions), happens partially in 3d
    arr_seg = arr_red.copy()
    arr_seg[arr_red<0.5] = 0
    arr_seg[arr_red>0.5] = 1
    mode = 'nearest'
    sobel_h = sobel(arr_seg, 0, mode=mode)  # horizontal gradient
    sobel_v = sobel(arr_seg, mode = mode)  # vertical gradient
    arr_edges = np.sqrt(sobel_h**2 + sobel_v**2)
    # arr_edges *= 0.2/arr_edges.max()  # normalization
    arr_edges[arr_edges>0]=2
    # arr_edges[arr_edges<0.07] = 0
    # print(rgb_image[259,168,1]+rgb_image[259,168,2])
    # combine attentions
    arr_att = arr_red + arr_edges
    arr_att = gaussian_filter(arr_att, sigma=3)      
    arr_att = gaussian_filter(arr_att, sigma=5)    
    arr_att[arr_att<0.31]=0
    return arr_att

# class FakeAttention

if __name__=="__main__":
    # arr = plt.imread("/root/thesis_ws/src/thesis/planner/view2.png")
    # arr_att = get_attention_map(arr)
    # plt.imshow(arr_att, cmap='jet')

    # plt.show()

    noname = NoName()
    # noname.run_animation()