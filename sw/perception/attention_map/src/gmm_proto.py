import numpy as np
import matplotlib.pyplot as plt
from sklearn.mixture import GaussianMixture
from matplotlib.patches import Ellipse

def generate_sample_data():
    np.random.seed(42)
    C = np.array([[0., -0.1, 0.2], 
                  [1.7, 0.4, 0.1], 
                  [0.1, 0.2, 1.5]])
    X = np.r_[np.dot(np.random.randn(300, 3), C),
              0.7 * np.random.randn(700, 3) + np.array([-6, 3, 0.5])]
    return X

def generate_data():
    np.random.seed(42)
    means = []
    X = np

def plot_gmm(gmm, X, ax):
    labels = gmm.predict(X)
    scatter = ax.scatter(X[:, 0], X[:, 1], c=X[:, 2], s=40, cmap='viridis', zorder=2)
    colorbar = plt.colorbar(scatter, ax=ax)
    colorbar.set_label('Intensity')
    
    w_factor = 0.2 / gmm.weights_.max()
    for pos, covar, w in zip(gmm.means_, gmm.covariances_, gmm.weights_):
        draw_ellipse(pos[:2], covar[:2, :2], ax, alpha=w * w_factor)

def draw_ellipse(position, covariance, ax, **kwargs):
    """Draw an ellipse with a given position and covariance"""
    if covariance.shape == (2, 2):
        U, s, Vt = np.linalg.svd(covariance)
        angle = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
        width, height = 2 * np.sqrt(s)
    else:
        angle = 0
        width, height = 2 * np.sqrt(covariance)
    
    for nsig in range(1, 4):
        ax.add_patch(Ellipse(position, nsig * width, nsig * height,
                             angle, **kwargs))

# Generate sample data
X = generate_sample_data()

# Fit a Gaussian Mixture Model with 2 components
gmm = GaussianMixture(n_components=2, covariance_type='full', random_state=42)
gmm.fit(X)

# Create a plot
fig, ax = plt.subplots(figsize=(8, 8))
plot_gmm(gmm, X, ax)
plt.title('Gaussian Mixture Model with Ellipses and Intensity')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.show()
