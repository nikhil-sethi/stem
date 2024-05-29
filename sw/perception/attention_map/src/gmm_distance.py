import numpy as np

def mahalanobis_distance(mu1, cov1, mu2):
    diff = mu2 - mu1
    cov_inv = np.linalg.inv(cov1)
    distance = np.sqrt(np.dot(np.dot(diff.T, cov_inv), diff))
    return distance

# Example means and covariances for two 4D Gaussians
mu1 = np.array([1.0, 2.0, 3.0, 4.0])
cov1 = np.array([[1.0, 0.1, 0.2, 0.1],
                 [0.1, 1.5, 0.1, 0.2],
                 [0.2, 0.1, 2.0, 0.3],
                 [0.1, 0.2, 0.3, 1.0]])

mu2 = np.array([1.5, 2.5, 3.5, 4.5])
cov2 = np.array([[1.2, 0.2, 0.1, 0.1],
                 [0.2, 1.3, 0.2, 0.1],
                 [0.1, 0.2, 1.8, 0.2],
                 [0.1, 0.1, 0.2, 1.1]])

distance = mahalanobis_distance(mu1, cov1, mu2)
print("Mahalanobis Distance:", distance)


def kl_divergence(mu1, cov1, mu2, cov2):
    cov2_inv = np.linalg.inv(cov2)
    term1 = np.trace(np.dot(cov2_inv, cov1))
    term2 = np.dot(np.dot((mu2 - mu1).T, cov2_inv), (mu2 - mu1))
    term3 = np.log(np.linalg.det(cov2) / np.linalg.det(cov1))
    k = len(mu1)
    return 0.5 * (term1 + term2 - k + term3)

kl_div = kl_divergence(mu1, cov1, mu2, cov2)
print("KL Divergence:", kl_div)

def bhattacharyya_distance(mu1, cov1, mu2, cov2):
    cov_mean = 0.5 * (cov1 + cov2)
    term1 = 0.125 * np.dot(np.dot((mu1 - mu2).T, np.linalg.inv(cov_mean)), (mu1 - mu2))
    term2 = 0.5 * np.log(np.linalg.det(cov_mean) / np.sqrt(np.linalg.det(cov1) * np.linalg.det(cov2)))
    return term1 + term2

bhat_dist = bhattacharyya_distance(mu1, cov1, mu2, cov2)
print("Bhattacharyya Distance:", bhat_dist)

