from scipy.stats import gamma
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 1)
import numpy as np

a = 2
mean, var, skew, kurt = gamma.stats(a, moments='mvsk')

x = np.linspace([0,0], [10,10], 100)
X,Y = np.meshgrid(x[:,0], x[:,1])
data = np.column_stack([X.ravel(), Y.ravel()])
print(data.shape)
Z = gamma.pdf(data, [2,5])
Z = Z[:,1]#*Z[:,1]
print(Z.shape)
Z = Z.reshape(100,100)
# print()
ax.imshow(Z)
# print(np.mgrid[0:10,0:10].reshape(10,10,2)[0,1])
# ax.plot(x, gamma.pdf(x, a),
#        'r-', lw=5, alpha=0.6, label='gamma pdf')

plt.show()