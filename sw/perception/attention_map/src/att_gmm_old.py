import rospy
from sklearn import mixture
import numpy as np

class GMMAttROS:
    def __init__(self):
        self.att_sub = rospy.Subscriber("attention_map/local", Image, self.att_callback, queue_size=10)
        self.img = np.zeros((480, 848))
        self.scale = 5

    def att_callback(self, msg):
        arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        data = self.sample_pdf(arr)
        gmm = self.fit_gmm(data)
        arr_gmm = self.draw_gmm(arr, gmm)
        
        att_gmm_msg = self.bridge.cv2_to_imgmsg(self.arr_gmm, encoding='bgr8')        
        att_gmm_msg.header = msg.header

        self.gmm_pub(att_gmm_msg)

    def fit_gmm(self):
        gmm = mixture.GaussianMixture(n_components=8, covariance_type="spherical", max_iter=100).fit(data)
        # for n in range(len(gmm.covariances_)):
        #     # get params
        #     mean = gmm.means_[n]
        #     cov = np.eye(gmm.means_.shape[1]) * gmm.covariances_[n]
        #     # print(gmm.covariances_)
        #     samples = np.random.multivariate_normal(mean, cov, 100).astype(int)
        #     # print(samples)
        #     avg_sal = np.mean(saliency_map[samples[:,0], samples[:,1]])
        #     # print(avg_sal)
        #     gmm.weights_[n] *= avg_sal
        #     # print(avg_sal, gmm.weights_[n])

        return gmm

    def sample_pdf(arr):
        # downsample for speed and consistency
        # scale = 5
        arr_ds = arr[::self.scale, ::self.scale]
        
        # the 2d map is the pdf itself. 
        # change the 0 to higher to make it more strict and localised to high probability regions
        pdf = arr_ds[arr_ds>0]
        
        # make the pdf sum up to 1. TODO check if this works because it might suppress high vals too much
        pdf = pdf/pdf.sum()
        
        # get valid pixels to sample from.
        X = np.argwhere(arr_ds>0)

        # sample randomly from the distribution to apply the GMM
        # this is a hack but it works pretty fast
        # otherwise you would have to perform a 2d curve fitting optimzation, which takes longer. TODO but still worth exploring more
        rng = np.random.default_rng()
        # data is scaled back to original size, so that guassian means and covs can be overlayed on the original img
        data =  self.scale*rng.choice(X, 1000, p = pdf, replace=False) 

        return data


    def 