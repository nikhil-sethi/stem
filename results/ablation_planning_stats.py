from utils import process_directory,get_mean_std, get_target_info_stat 
from plotting import plot_mean_std,  plot_tivs
import matplotlib.pyplot as plt
import numpy as np
plt.style.use('ggplot')
np.set_printoptions(linewidth=200, precision=3)

# load data for statistics
world = "cave" 

if world=="earthquake":
    MAX_ENTROPY = 115703.375  # earthquake: 115703.375, cave:  1208686.75
    lambda_min = 0.02 # detection threshold
elif world=="cave":
    MAX_ENTROPY = 1208686.75
    lambda_min = 0.015 # detection threshold


data_greedy = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/GREEDY")
data_lkh = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/LKH")
data_motsp = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/SS+AP")
# data_ss = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/SS")

mu_wig_greedy, std_wig_greedy =  get_mean_std(data_greedy, idx=5) 
mu_wig_lkh, std_wig_lkh =  get_mean_std(data_lkh, idx=5)
mu_wig_motsp, std_wig_motsp =  get_mean_std(data_motsp, idx=5) 

# INFORMATION GAIN
fig, ax1 = plt.subplots()
plot_mean_std(mu_wig_greedy/MAX_ENTROPY, std_wig_greedy/MAX_ENTROPY, ax1, color="salmon", label="GREEDY")
plot_mean_std(mu_wig_lkh/MAX_ENTROPY, std_wig_lkh/MAX_ENTROPY, ax1, color="green", label="TSP")
plot_mean_std(mu_wig_motsp/MAX_ENTROPY, std_wig_motsp/MAX_ENTROPY, ax1, color="blue", label="WMLP")
ax1.set_title("Weighted information gain vs. Time(s)", fontsize=17)
ax1.set_ylabel("Information gain", fontsize=15,color='black')
ax1.set_xlabel("Time(s)", fontsize=15,color='black')
ax1.legend(prop={'size': 15})
plt.ticklabel_format(axis='both', style='sci', scilimits=(0,4))
ax1.tick_params(axis='x', labelsize=15)
ax1.tick_params(axis='y', labelsize=15)


# TIME IN VIEW
fig2, ax2 = plt.subplots()
ax2.set_title("Class vs. Time in view ",fontsize=17,color="black")
ax2.set_xlabel("Time in view (s)",fontsize=15,color="black")
# plot_tivs(ax2, data_greedy, 1, color="salmon", world=world) 
# plot_tivs(ax2, data_lkh, 2, color="green", world=world) 
# plot_tivs(ax2, data_motsp, 3, color="blue", world=world) 
from matplotlib.lines import Line2D

custom_lines = [Line2D([0], [0], color="lightsalmon", lw=4),
                Line2D([0], [0], color="lightgreen", lw=4),
                Line2D([0], [0], color="lightblue", lw=4)]

ax2.legend(custom_lines, ['Greedy ($\mu \pm \sigma$)', 'TSP ($\mu \pm \sigma$)', 'WMLP ($\mu \pm \sigma$)'],prop={'size': 15}, loc="upper right" )


# TARGET INFO
print("Greedy stats" , *get_target_info_stat(data_greedy, threshold=lambda_min, norm_factor=MAX_ENTROPY))
print("LKH stats" , *get_target_info_stat(data_lkh, threshold=lambda_min, norm_factor=MAX_ENTROPY)) 
print("MOTSP stats" , *get_target_info_stat(data_motsp, threshold=lambda_min, norm_factor=MAX_ENTROPY) )

plt.show()


"""

"""