import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sklearn import mixture, cluster
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import matplotlib.pyplot as plt
import time
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_matrix, random_rotation_matrix
from sklearn.mixture._gaussian_mixture import _check_weights, _compute_precision_cholesky
from scipy import linalg


cmap = plt.get_cmap("tab20")
cmap_colors = np.array(cmap.colors)
cov_type = "full"


def bhattacharyya_distance(mu1, cov1, mu2, cov2):
    cov_mean = 0.5 * (cov1 + cov2)
    term1 = 0.125 * np.dot(np.dot((mu1 - mu2).T, np.linalg.inv(cov_mean)), (mu1 - mu2))
    term2 = 0.5 * np.log(np.linalg.det(cov_mean) / np.sqrt(np.linalg.det(cov1) * np.linalg.det(cov2)))
    return term1 + term2


def compute_precision_cholesky(covariance, covariance_type):
    """Compute the Cholesky decomposition of the 1 covariance.

    Parameters
    ----------
    covariances : array-like
        The covariance matrix of the current components.
        The shape depends of the covariance_type.

    covariance_type : {'full', 'tied', 'diag', 'spherical'}
        The type of precision matrices.

    Returns
    -------
    precisions_cholesky : array-like
        The cholesky decomposition of sample precisions of the current
        components. The shape depends of the covariance_type.
    """
    estimate_precision_error_message = (
        "Fitting the mixture model failed because some components have "
        "ill-defined empirical covariance (for instance caused by singleton "
        "or collapsed samples). Try to decrease the number of components, "
        "or increase reg_covar."
    )

    if covariance_type == "full":
        n_features, _ = covariance.shape
        # precisions_chol = np.empty((n_components, n_features, n_features))
        # for k, covariance in enumerate(covariances):
        try:
            cov_chol = linalg.cholesky(covariance, lower=True)
        except linalg.LinAlgError:
            raise ValueError(estimate_precision_error_message)
        precision_chol = linalg.solve_triangular(
            cov_chol, np.eye(n_features), lower=True
        ).T
    elif covariance_type == "tied":
        n_features = covariance.shape
        try:
            cov_chol = linalg.cholesky(covariance, lower=True)
        except linalg.LinAlgError:
            raise ValueError(estimate_precision_error_message)
        precision_chol = linalg.solve_triangular(
            cov_chol, np.eye(n_features), lower=True
        ).T
    else:
        if np.any(np.less_equal(covariance, 0.0)):
            raise ValueError(estimate_precision_error_message)
        precision_chol = 1.0 / np.sqrt(covariance)
    return precision_chol




def get_orientation_and_scale(covariance):
    eigvals, eigvecs = np.linalg.eigh(covariance)
        
    rotmat = random_rotation_matrix()
    # rotmat = rotmat/np.linalg.norm(rotmat, axis=0)
    rotmat[:3, :3] = eigvecs
    # print(rotmat.shape)
    quat = quaternion_from_matrix(rotmat)
    quat = quat/np.linalg.norm(quat)

    return quat, eigvals

def get_ellipsoids(gmm, sigma=2):
    """Get sigma confidence ellipse"""
    poses = []
    scales = []
    for i in range(len(gmm.means_)):
        mean = gmm.means_[i]
        cov = gmm.covariances_[i]
        if gmm.covariance_type =="spherical":
            cov = np.eye(gmm.means_.shape[1]) * cov
        # print(cov)
        quat, scale = get_orientation_and_scale(cov[:3, :3])
        pose = Pose(position = mean, orientation = quat)
        scale = np.sqrt(5.991 * scale)
        scales.append(scale)
        poses.append(pose)
        # self.create_marker(position, )
            
    return poses, scales

def create_gmm_marker(pose, scale, id, color, type=Marker.SPHERE):
    marker = Marker ()
    marker.header.frame_id ="world";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "ellipse" + str(id);
    marker.id = id;
    marker.type = type
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

    marker.color.a = color[3]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    return marker

def create_arrow_marker(pose, scale, id, color):
    marker = Marker ()
    marker.header.frame_id ="world";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "ellipse" + str(id);
    marker.id = id;
    marker.type = Marker.ARROW
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

    marker.scale.x = 0.5
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    marker.color.a = color[3]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    return marker

def create_ellipse_mesh(mean, cov):
    # if cov.shape = (1,1)
    print(cov)
    cov = cov.reshape(4,4)
    # get scale and transform from distribution
    eigvals, eigvecs = np.linalg.eigh(cov[:3, :3])

    # create mesh
    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    mesh_sphere.compute_vertex_normals()
    X = np.asarray(mesh_sphere.vertices)

    # scale
    scale = np.sqrt(5.991 * eigvals)
    X = X*scale
    # print(scale)
    # transform
    T = np.eye(4)
    T[:3, :3] = eigvecs
    T[:3, -1]  = mean[:3]
    X_homo = np.column_stack([X, np.ones(X.shape[0])])

    X = X_homo@T.T

    # reset
    mesh_sphere.vertices = o3d.utility.Vector3dVector(X[:, :3])

    return mesh_sphere

def to_o3d(data):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:, :3])
    # intensity_max = 
    colors = plt.get_cmap("tab20")(data[:, -1] / 5)
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return pcd

def merge_gaussians(pi1, mu1, cov1, n1, pi2, mu2, cov2, n2):
    
    _n1 = pi1*n1 # support of gaussian 1
    _n2 = pi2*n2 # support of gaussian 2
    _n =  _n1 + _n2 # combined support

    pi = (_n1 + _n2)/(n1 + n2) # combined rep = combined_support/original_support
   
     # Calculate the new mean
    mu = (_n1 * mu1 + _n2 * mu2) / _n
    
    # Calculate the new covariance
    cov = (_n1 * (cov1 + np.outer(mu1 - mu, mu1 - mu)) + 
           _n2 * (cov2 + np.outer(mu2 - mu, mu2 - mu))) / _n
    
    return pi, mu, cov

def merge_as_mixture(pi1, mu1, cov1, pi2, mu2, cov2):
    pi_sum = pi1 + pi2
    return [pi1/pi_sum, pi2/pi_sum], [mu1, mu2], [cov1, cov2]

def merge_into_mixture(gmm, n, pi2, mu2, cov2, prec_chol2, n2):
    
    gmm.means_ = np.append(gmm.means_, [mu2], axis=0)
    gmm.covariances_ = np.append(gmm.covariances_, [cov2], axis=0)
    gmm.precisions_cholesky_ = np.append(gmm.precisions_cholesky_, [prec_chol2], axis=0)
    # print(gmm.covariances_)
    # renormalise weights
    n_sum = n + pi2*n2
    # print(n_sum, n, n2)
    # in place list change
    for i in range(len(gmm.weights_)):
        gmm.weights_[i] = gmm.weights_[i]*n/n_sum 
    pi2_norm = pi2*n2/n_sum
    
    gmm.weights_ = np.append(gmm.weights_, pi2_norm)


    # print(pis, sum(pis), pi2_norm)    
     

def mc_kl_div(gmm1, gmm2):
    pdf1 = gmm1.sample(100)
    pdf2 = gmm2.sample(100)

    kl_div = np.sum(np.log(pdf1/pdf2))

    return kl_div

def are_gaussians_similar(mu1, cov1, mu2, cov2):

    # mean checkn
    # if np.linalg.norm(mu1[:3] - mu2[:3]) > 0.5 or abs(mu1[-1] - mu2 [-1]) > 0.5:
    #     return False 

    if bhattacharyya_distance(mu1, cov1, mu2, cov2) < 1:
        return True



    # normal check
    quat1, _ = get_orientation_and_scale(cov1[:3, :3])
    quat2, _ = get_orientation_and_scale(cov2[:3, :3])
    eps = 0.3
    # if 1 - abs(quat1.dot(quat2)) < eps:
    #     return True

    # IOU

    # KL divergence

    
    return False

def cluster_extraction(data):
    dbscan = cluster.DBSCAN(eps=0.13, min_samples=7)
    labels = dbscan.fit_predict(data)

    max_label = labels.max()
    clusters = [data[labels==i] for i in range(max_label+1)]
    # print(f"point cloud has {max_label + 1} clusters")

    cluster_means = np.array([np.mean(cluster, axis=0) for cluster in clusters])

    return cluster_means, labels

def check_normalised_weights(gmm):
    return abs(np.sum(gmm.weights) - 1) < 0.001


class GIFMap():
    def __init__(self) -> None:
        self.global_gmm = mixture.GaussianMixture(n_components=0, covariance_type=cov_type)
        self.global_gmm.means_ = np.empty(shape=(0,4))
        self.global_gmm.covariances_ = np.empty(shape=(0,4,4))
        self.global_gmm.precisions_cholesky_ = np.empty(shape=(0,4,4))
        self.global_gmm.weights_ = np.empty(shape=(0,))
        self.global_support = 0
        self.local_gmm = self.global_gmm # just for initing

        self.gmm_ell_pub = rospy.Publisher("/attention_map/local/ellipses", Marker, queue_size=10)
        self.cloud_pub = rospy.Publisher("/attention_map/local/sampled", PointCloud2, queue_size=10)

        msg = rospy.wait_for_message("/attention_map/local/filtered", PointCloud2)
        self.cloud_callback(msg)

        self.cloud_sub = rospy.Subscriber("/attention_map/local/filtered", PointCloud2, self.cloud_callback)
        # self.vis_timer = rospy.Timer(rospy.Duration(0.1), self.vis_timer_callback)
        # init variables for timer
        # self.local_gmm = self.infer_gmm(self.data)

    def update_timer(self, event):
        pass


    def vis_timer_callback(self, event):
        # for i in range(15):
        #     self.publish_empty_marker(i)
        #     self.publish_empty_marker(i*10)
        #     self.publish_empty_marker(i*20)


        # PLOT ELLIPSES
        # global GMM
        poses, scales = get_ellipsoids(self.global_gmm)

        for i in range(len(poses)):
            self.publish_marker(poses[i], scales[i], i+1, color=(0,1,0,0.6))
        

        # local GMM
        poses, scales = get_ellipsoids(self.local_gmm)

        for i in range(len(poses)):
            self.publish_marker(poses[i], scales[i], (i+1)*300, color=(0,0,1,0.3))
    
        # PLOT INFERENCE SAMPLES
        samples = self.sample(self.global_gmm)
        cloud_msg = pc2.create_cloud(self.header, self.fields, samples)

        self.cloud_pub.publish(cloud_msg)

    def cloud_callback(self, msg):
        # self.data = np.array(list(pc2.read_points(msg)))
        start = time.perf_counter()
        data = np.array(list(pc2.read_points(msg)))
        # print(data)
        if data.size != 0:
            data[:, -1] = np.round(data[:, -1])

            # dbscan on depth intensity data. 
            cluster_means, labels = cluster_extraction(data)    

            if cluster_means.size != 0:
                self.local_gmm = self.create_gmm(data, cluster_means)
                local_support = len(data)
                self.update_gmm(self.local_gmm, local_support)

        print(self.global_gmm.n_components)
        print(time.perf_counter()-start)
        
        self.header = msg.header
        self.fields = msg.fields
        self.vis_timer_callback(None)

        # vis will use them
        
       

    def publish_empty_marker(self, i):
        marker = Marker ()
        marker.header.frame_id ="world";
        marker.header.stamp = rospy.Time.now ()
        marker.ns = "ellipse" + str(i);
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE

        self.gmm_ell_pub.publish(marker)

    def publish_marker(self, pose, scale, id, color=(0,1,0,0.4)):
        marker = create_gmm_marker(pose, scale, id, color)
        self.gmm_ell_pub.publish(marker)

        # marker = create_arrow_marker(pose, scale, id*10, color)
        # self.gmm_ell_pub.publish(marker)

        # marker = create_gmm_marker(pose, scale, id*20, alpha, type=Marker.CUBE)
        # self.gmm_ell_pub.publish(marker)

    def create_gmm(self, data, means_init):
        # Create GMM from cluster centroids
        gmm = mixture.GaussianMixture(means_init=means_init, n_components=len(means_init), covariance_type=cov_type, reg_covar=1e-3)
        gmm.fit(data)
        return gmm

    def update_gmm(self, new_gmm, local_support):
        _check_weights(new_gmm.weights_, new_gmm.n_components)
        
        old_gmm = self.global_gmm
        n_components_old = old_gmm.n_components # save because it might change
        
        for i in range(new_gmm.n_components):
            w_i, mu_i, cov_i, prec_i = new_gmm.weights_[i], new_gmm.means_[i], new_gmm.covariances_[i], new_gmm.precisions_cholesky_[i]
            if w_i<0.1:
                continue
            merged = False

            for j in range(n_components_old):
                w_j, mu_j, cov_j = old_gmm.weights_[j], old_gmm.means_[j], old_gmm.covariances_[j]
                if are_gaussians_similar(mu_i, cov_i, mu_j, cov_j):
                    # _, old_gmm.means_[j], old_gmm.covariances_[j] = merge_gaussians(w_j, mu_j, cov_j, self.global_support, w_i, mu_i, cov_i, local_support)
                    # print("merge")
                    # old_gmm.weights_ = list(old_gmm.weights_/np.linalg.norm(old_gmm.weights_))
                    # old_gmm.precisions_cholesky_[j] = compute_precision_cholesky(old_gmm.covariances_[j], old_gmm.covariance_type)
                    merged = True
                    break

            if not merged: # guassian is unique, add it as a component
                merge_into_mixture(old_gmm, self.global_support, w_i, mu_i, cov_i, prec_i, local_support)
                old_gmm.n_components += 1
                self.global_support += w_i*local_support

        _check_weights(old_gmm.weights_, old_gmm.n_components)

        # normalise weights
        """
        for each g1 in new_gmm
            merged = false
            for each g2 in old_gmm
                if  nbrs(g1, g2):
                    g2.merge(g1) # in place merge
                    g2 = merged
            if not merged // no candidate to merge into
                old_gmm.merge(g1)
        
        """

    def sample(self, gmm):
        samples, _ = gmm.sample(500) # sample from the combined mixture.
        ll = gmm.score_samples(samples)    
        ll = (ll - ll.min())/(ll.max() - ll.min()) # normalise likelihood. treat it as probability

        samples = samples[ll>0.5]
        
        # samples = samples[samples[:, -1]>0.5] # priority: only keep objects of interest
        # samples[:, -1]*=ll[ll>0.7] # confidence: only keep data points which we make a lot of observations for
        
        return samples

"""
update data list
create gmm
update existing gmm with incoming data

"""

def test_merge():
    gif = GIFMap()
    # rospy.spin()
    old_gmm = gif.global_gmm

    new_gmm = mixture.GaussianMixture(n_components=2)
    new_gmm.weights_ = np.array([0.6, 0.4])
    n_i = 2000
    new_gmm.means_ = np.array([[1.5, 3.0, 1.0, 4], 
                               [1.0, 2.0, 1.0, 5]])
    a = np.random.rand(4,4)
    b = np.random.rand(4,4)
    new_gmm.covariances_ = np.array([a@a.T,
                                    b@b.T])
    new_gmm.precisions_cholesky_ = _compute_precision_cholesky(new_gmm.covariances_, "full")

    gif.update_gmm(new_gmm, n_i)
    print(gif.global_gmm.means_)
    gif.update_gmm(new_gmm, n_i)
    print(gif.global_gmm.means_)

def main():
    rospy.init_node("gif_node")
    gif = GIFMap()
    rospy.spin()


if __name__=="__main__":
    main()
    # test_merge()
    
    
