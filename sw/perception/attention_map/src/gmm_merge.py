import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def merge_gaussians(pi1, mu1, cov1, n1, pi2, mu2, cov2, n2):
    # Calculate the combined weight
    
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

def merge_gaussians2(pi1, mu1, cov1, n1, pi2, mu2, cov2, n2):
    
    # Calculate the combined weight
    
    _n1 = pi1*n1 # support of gaussian 1
    _n2 = pi2*n2 # support of gaussian 2
    _n =  _n1 + _n2 # combined support
    n = n1 + n2

    w1 = _n1/n
    mom1 = _n1*mu1
    MOM1 = _n1*(cov1 + np.outer(mu1, mu1))

    w2 = _n2/n
    mom2 = _n2*mu2
    MOM2 = _n2*(cov2 + np.outer(mu2, mu2))

    mom = mom1 + mom2
    MOM = MOM1 + MOM2

    # recover
    mu =  mom/_n
    cov = MOM/_n - np.outer(mu, mu)
    pi = w1 + w2

    return pi, mu, cov


def plot_gaussian(ax, mean, cov, label, color):
    # Plot the mean as a point
    ax.plot(mean[0], mean[1], 'o', color=color, label=label)
    
    # Plot the covariance as an ellipse
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = eigenvalues.argsort()[::-1]
    eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]
    angle = np.degrees(np.arctan2(*eigenvectors[:,0][::-1]))
    width, height = 2 * np.sqrt(eigenvalues)
    
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=angle, edgecolor=color, fc='None', lw=2)
    ax.add_patch(ellipse)

# Example parameters for the two Gaussians
pi1 = 0.4
n1 = 1000
mu1 = np.array([1.0, 2.0])
cov1 = np.array([[0.5, 0.2], [0.2, 0.3]])

pi2 = 0.6
n2 = 2000
mu2 = np.array([1.5, 3.0])
cov2 = np.array([[0.7, 0.4], [0.4, 0.5]])

# Merging the two Gaussians
pi, mu, cov = merge_gaussians2(pi1, mu1, cov1, n1, pi2, mu2, cov2, n2)
print(pi, mu, cov)
# Plot the original and merged Gaussians
fig, ax = plt.subplots()

plot_gaussian(ax, mu1, cov1, 'Gaussian 1', 'blue')
plot_gaussian(ax, mu2, cov2, 'Gaussian 2', 'green')
plot_gaussian(ax, mu, cov, 'Merged Gaussian', 'red')

ax.set_xlim(-1, 5)
ax.set_ylim(0, 6)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.legend()
ax.set_title('Original and Merged Gaussians')
plt.show()
