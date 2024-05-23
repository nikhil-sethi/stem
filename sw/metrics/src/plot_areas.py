import matplotlib.pyplot as plt
import numpy as np

plt.style.use('ggplot')
fig, ax = plt.subplots()

data = np.loadtxt("/root/thesis_ws/src/thesis/sw/metrics/src/areas.csv", delimiter=",")

times = data[:,-1]
areas = data[:,:-1]
# areas[areas>0] = 1
plt.plot(times, areas, linewidth=3)
plt.legend(("1","2","3","4","5"))
plt.show()