import rospy
from sklearn import mixture, cluster
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, random_rotation_matrix

class GMMAttROS:
    def __init__(self):
        rospy.init_node("tes")
        self.att_sub = rospy.Subscriber("/attention_map/3d", PointCloud2, self.att_callback, queue_size=10)
        self.gmm_pub = rospy.Publisher("/attention_map/gmm/local", PointCloud2, queue_size=10)
        self.gmm_ell_pub = rospy.Publisher("/attention_map/gmm/local/ellipses", Marker, queue_size=10)
        self.gmm_global_pub = rospy.Publisher("/attention_map/gmm/global", PointCloud2, queue_size=10)

        # self.means = [[3, 4, 5, 1],
        #               [1, 1, 1, 0.5]]
        # self.covs =  [
        #     np.eye(4), np.eye(4)
        # ]
        # self.data = np.random.multivariate_normal(mean=self.means, cov=self.covs, size=500)
        # self.gmm_global =  mixture.GaussianMixture(n_components=10, covariance_type="full", max_iter=100).fit(data)
        self.window = 5
        self.count = 0
        self.local_gmms = []
        rospy.spin()

    def att_callback(self, msg):
        # data = msg.data
        data = np.array(list(pc2.read_points(msg)))
        # data = self.data
        # print(data)

        # windowing 
        # cluster only spatial data using kmeans 
        kmeans = cluster.KMeans(n_clusters=3, n_init='auto')
        labels = kmeans.fit_predict(data[:, :-1])

        # fit gmm on each window

        self.window_gmms = []
        for i in range(3):
            data_k = data[labels==i]
            gmm_k = self.fit_gmm(data_k)
            self.window_gmms.append(gmm_k)

        # gmm = self.fit_gmm(data)
        # merge all windows
        self.local_gmm = self.merge_gmms(self.window_gmms)
        self.local_gmms.append(self.local_gmm)

        if len(self.local_gmms)>self.window:
            self.local_gmms.pop(0)
        
        # merges past and current gmm for global gmm
        # self.update_global_gmm()
        self.gmm_global = self.merge_gmms(self.local_gmms)
        
        poses, scales = self.get_ellipsoids(self.gmm_global)
        # clear all markers
        for i in range(15*self.window):
            self.publish_empty_marker(i)

        for i in range(len(poses)):
            self.publish_marker(poses[i], scales[i], i, self.gmm_global.weights_[i])
        # # self.merge_gmm(gmm, self.gmm_global)
        
        samples = self.sample(self.gmm_global)

        # # print(gmm.means_)
        cloud_msg = pc2.create_cloud(msg.header, msg.fields, samples)

        self.gmm_pub.publish(cloud_msg)

    def publish_empty_marker(self, i):
        marker = Marker ()
        marker.header.frame_id ="world";
        marker.header.stamp = rospy.Time.now ()
        marker.ns = "ellipse" + str(i);
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE

        self.gmm_ell_pub.publish(marker)

    def get_ellipsoids(self, gmm, sigma=2):
        """Get sigma confidence ellipse"""
        poses = []
        scales = []
        for i in range(len(gmm.means_)):
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

    def sample(self, gmm):
        samples, _ = gmm.sample(500) # sample from the combined mixture.
        ll = gmm.score_samples(samples)    
        ll = (ll - ll.min())/(ll.max() - ll.min()) # normalise likelihood. treat it as probability

        samples = samples[ll>0.5]
        
        samples = samples[samples[:, -1]>0.5] # priority: only keep objects of interest
        # samples[:, -1]*=ll[ll>0.7] # confidence: only keep data points which we make a lot of observations for
        
        return samples

    def fit_gmm(self, data):
        gmm = mixture.GaussianMixture(n_components=8, covariance_type="full",init_params='k-means++', reg_covar=1e-5).fit(data)

        return gmm
    
    def update_global_gmm(self):
        """Merge global gmm with local update"""
        # combined_means = np.vstack([gmm.means_ for gmm in self.gmms])
        # combined_covariances = np.vstack([gmm.covariances_ for gmm in self.gmms])
        # combined_weights = np.hstack([gmm.weights_ for gmm in self.gmms])
        # combined_prec_chol = np.vstack([gmm.precisions_cholesky_ for gmm in self.gmms])
        combined_covariances = []
        combined_means = []
        combined_prec_chol = []
        combined_weights = []
        for gmm in self.gmms:
            for i in range(len(gmm.weights_)):
                if gmm.weights_[i]<0.02:
                    continue
                combined_means.append(gmm.means_[i])
                combined_covariances.append(gmm.covariances_[i])
                combined_weights.append(gmm.weights_[i])
                combined_prec_chol.append(gmm.precisions_cholesky_[i])

        # Normalize weights to sum to 1
        combined_weights /= np.sum(combined_weights)

        self.gmm_global = mixture.GaussianMixture(n_components=len(combined_weights), covariance_type='full')
        self.gmm_global.means_ = np.array(combined_means)
        self.gmm_global.covariances_ = np.array(combined_covariances)
        self.gmm_global.precisions_cholesky_ = np.array(combined_prec_chol)
        self.gmm_global.weights_ = np.array(combined_weights)

    def merge_gmms(self, gmms):
        """Merge a list of sklearn instances"""

        combined_covariances = []
        combined_means = []
        combined_prec_chol = []
        combined_weights = []
        for gmm in gmms: 
            for i in range(len(gmm.weights_)):
                if gmm.weights_[i]<0.03:
                    continue
                combined_means.append(gmm.means_[i])
                combined_covariances.append(gmm.covariances_[i])
                combined_weights.append(gmm.weights_[i])
                combined_prec_chol.append(gmm.precisions_cholesky_[i])

        # Normalize weights to sum to 1
        combined_weights /= np.sum(combined_weights)

        gmm_combined = mixture.GaussianMixture(n_components=len(combined_weights), covariance_type='full')
        gmm_combined.means_ = np.array(combined_means)
        gmm_combined.covariances_ = np.array(combined_covariances)
        gmm_combined.precisions_cholesky_ = np.array(combined_prec_chol)
        gmm_combined.weights_ = np.array(combined_weights)

        return gmm_combined
     


    # def merge_gmms(self, other):
        
    # def global_gmm_timer(self, event):
    #     """publish the global mixture"""
    #     samples = self.infer_gmm(self.global_gmm)
    #     cloud_msg = pc2.create_cloud(msg.header, msg.fields, samples)

    #     self.gmm_pub.publish(cloud_msg)

if __name__=="__main__":
    gmm = GMMAttROS()