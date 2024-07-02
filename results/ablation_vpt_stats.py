from utils import process_directory,get_mean_std, get_target_info_stat 
from plotting import plot_mean_std,  plot_tivs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.style.use('ggplot')
np.set_printoptions(linewidth=200, precision=3)

# load data for statistics
world = "earthquake" 

if world=="earthquake":
    MAX_ENTROPY = 115703.375  # earthquake: 115703.375, cave:  1208686.75
    lambda_min = 0.02 # detection threshold
elif world=="cave":
    MAX_ENTROPY = 1208686.75
    lambda_min = 0.015 # detection threshold


data_frontier = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/COVERAGE")
data_ss = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/SS")
data_ss_ap = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/SS+AP")

mu_wif_frontier, std_wif_frontier =  get_mean_std(data_frontier, idx=5) 
mu_wif_ss_ap, std_wif_ss_ap =  get_mean_std(data_ss_ap, idx=5) 
mu_wif_ss, std_wif_ss =  get_mean_std(data_ss, idx=5) 

# INFORMATION GAIN
fig, ax1 = plt.subplots()
plot_mean_std(mu_wif_frontier/MAX_ENTROPY, std_wif_frontier/MAX_ENTROPY, ax1, color="salmon", label="COVERAGE")
plot_mean_std(mu_wif_ss/MAX_ENTROPY, std_wif_ss/MAX_ENTROPY, ax1, color="green", label="SS")
plot_mean_std(mu_wif_ss_ap/MAX_ENTROPY, std_wif_ss_ap/MAX_ENTROPY, ax1, color="blue", label="SS+AP")
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
plot_tivs(ax2, data_frontier, 1,color="lightsalmon",max_bars=3, world=world) 
plot_tivs(ax2, data_ss, 2,color="lightcoral",max_bars=3, world=world) 
plot_tivs(ax2, data_ss_ap, 3,color="lightskyblue",max_bars=3, world=world) 
from matplotlib.lines import Line2D

custom_lines = [Line2D([0], [0], color="lightsalmon", lw=4),
                Line2D([0], [0], color="lightcoral", lw=4),
                Line2D([0], [0], color="lightskyblue", lw=4)]

ax2.legend(custom_lines, ['COVERAGE ($\mu \pm \sigma$)', 'SS ($\mu \pm \sigma$)', 'SS+AP ($\mu \pm \sigma$)'],prop={'size': 15}, loc="center right" )

# NUMBER stats

# print("FUEL stats" , 109294.41499999998/EARTHQUAKE_MAX_ENTROPY, 1176.4867547876624/EARTHQUAKE_MAX_ENTROPY )
print("COVERAGE stats", *get_target_info_stat(data_frontier, threshold=lambda_min, norm_factor=MAX_ENTROPY))   # the relaxed threshold is just to get SOME output. Please dont count the target results for this threshold 
print("SS stats" , *get_target_info_stat(data_ss, threshold=lambda_min, norm_factor=MAX_ENTROPY) )
print("SS_AP stats" , *get_target_info_stat(data_ss_ap, threshold=lambda_min, norm_factor=MAX_ENTROPY)) 

plt.show()
