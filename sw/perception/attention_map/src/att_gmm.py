import rospy
from sklearn import mixture
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import random_rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

def get_ellipsoids(gmm, sigma=2):
    """Get sigma confidence ellipse"""
    poses = []
    scales = []
    for i in range(len(gmm.means_)):
        # if gmm.weights_[i]<0.01:
        #     continue
        mean = gmm.means_[i]
        cov = gmm.covariances_[i]

        eigvals, eigvecs = np.linalg.eigh(cov[:3, :3])
        
        rotmat = random_rotation_matrix()
        # rotmat = rotmat/np.linalg.norm(rotmat, axis=0)
        rotmat[:3, :3] = eigvecs
        # print(rotmat.shape)
        quat = quaternion_from_matrix(rotmat)
        quat = quat/np.linalg.norm(quat)
        pose = Pose(position = mean, orientation = quat)
        scale = np.sqrt(5.991 * eigvals)
        scales.append(scale)
        poses.append(pose)
        # self.create_marker(position, )
        
    return poses, scales



class GMMAttROS:
    def __init__(self):
        rospy.init_node("tes")
        self.att_sub = rospy.Subscriber("/attention_map/3d", PointCloud2, self.att_callback, queue_size=10)
        self.gmm_pub = rospy.Publisher("/attention_map/global", PointCloud2, queue_size=10)
        self.gmm_ell_pub = rospy.Publisher("/attention_map/gmm/local/ellipses", Marker, queue_size=10)

        self.img = np.zeros((480, 848))
        self.scale = 5
        # self.means  np.a
        rospy.spin()

    def att_callback(self, msg):
        # data = msg.data
        data = np.array(list(pc2.read_points(msg)))
        # print(data)
        gmm = self.fit_gmm(data)
        # print(gmm.means_)

        poses, scales = get_ellipsoids(gmm)
        # clear all markers
        for i in range(20):
            self.publish_empty_marker(i)
        print(len(poses))
        for i in range(len(poses)):
            self.publish_marker(poses[i], scales[i], i, gmm.weights_[i])


        samples, _ = gmm.sample(500) # sample from the combined mixture.
        # ll = gmm.score_samples(samples)
        
        # ll = (ll - ll.min())/(ll.max() - ll.min()) # normalise likelihood. treat it as probability
        # print(ll)
        # ll>0.5
        
        # samples = samples[ll>0.7]
        
        samples = samples[samples[:, -1]>0.5] # priority: only keep objects of interest
        # samples[:, -1]*=ll[ll>0.7] # confidence: only keep data points which we make a lot of observations for
        
        cloud_msg = pc2.create_cloud(msg.header, msg.fields, samples)

        self.gmm_pub.publish(cloud_msg)

    def fit_gmm(self, data):
        try:
            gmm = mixture.GaussianMixture(n_components=20, covariance_type="full", max_iter=100, means_init=self.means_last, precisions_init=self.precs_last, weights_init=self.weights_last, reg_covar=5e-5).fit(data)
        except:
            gmm = mixture.GaussianMixture(n_components=20, covariance_type="full", max_iter=100, reg_covar=1e-4).fit(data)
            self.means_last = gmm.means_
            self.precs_last = gmm.precisions_
            self.weights_last = gmm.weights_
        return gmm
    
    def publish_marker(self, pose, scale, id, alpha):
        marker = Marker ()
        marker.header.frame_id ="world";
        marker.header.stamp = rospy.Time.now ()
        marker.ns = "ellipse" + str(id);
        marker.id = id;
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # position = pose.position
        marker.pose.position.x = pose.position[0]
        marker.pose.position.y = pose.position[1]
        marker.pose.position.z = pose.position[2]

        quat = pose.orientation
        
        marker.pose.orientation.x =quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
    
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        orientation = pose

        self.gmm_ell_pub.publish(marker)
    
    def publish_empty_marker(self, i):
        marker = Marker ()
        marker.header.frame_id ="world";
        marker.header.stamp = rospy.Time.now ()
        marker.ns = "ellipse" + str(i);
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE

        self.gmm_ell_pub.publish(marker)

if __name__=="__main__":
    gmm = GMMAttROS()