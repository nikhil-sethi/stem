from utils import process_directory,get_mean_std, get_target_info_stat 
from plotting import plot_mean_std,  plot_tivs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

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

data_fuel = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/FUEL")
data_fuel_f0_i10 = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/FUEL_MOD")
data_vsep = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/VSEP")
data_ss_ap = process_directory(f"/root/thesis_ws/src/thesis/results/data/{world}/SS+AP")

mu_wif_fuel, std_wif_fuel =  get_mean_std(data_fuel, idx=5) 
mu_wif_fuel_f0_i10, std_wif_fuel_f0_i10, =  get_mean_std(data_fuel_f0_i10, idx=5) 
mu_wif_vsep, std_wif_vsep =  get_mean_std(data_vsep, idx=5) 
mu_wif_ss_ap, std_wif_ss_ap =  get_mean_std(data_ss_ap, idx=5) 


# INFORMATION GAIN
fig, ax1 = plt.subplots()
plot_mean_std(mu_wif_fuel/MAX_ENTROPY, std_wif_fuel/MAX_ENTROPY, ax1, color="green", label="FUEL")
plot_mean_std(mu_wif_fuel_f0_i10/MAX_ENTROPY, std_wif_fuel_f0_i10/MAX_ENTROPY, ax1, color="darkorange", label="FUEL-mod")
plot_mean_std(mu_wif_vsep/MAX_ENTROPY, std_wif_vsep/MAX_ENTROPY, ax1, color="red", label="VSEP")
plot_mean_std(mu_wif_ss_ap/MAX_ENTROPY, std_wif_ss_ap/MAX_ENTROPY, ax1, color="blue", label="Ours")
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
plot_tivs(ax2, data_fuel, 1, color="lightgreen", max_bars=4, world=world)
plot_tivs(ax2, data_fuel_f0_i10, 2, color="lightsalmon", max_bars=4, world=world)  
plot_tivs(ax2, data_vsep, 3, color="lightcoral", max_bars=4, world=world) 
plot_tivs(ax2, data_ss_ap, 4, color="lightskyblue", max_bars=4, world=world) 
from matplotlib.lines import Line2D

custom_lines = [Line2D([0], [0], color="lightgreen", lw=6),
                Line2D([0], [0], color="lightsalmon", lw=6),
                Line2D([0], [0], color="lightcoral", lw=6),
                Line2D([0], [0], color="lightskyblue", lw=6)]

ax2.legend(custom_lines, ['FUEL ($\mu \pm \sigma$)','FUEL-mod ($\mu \pm \sigma$)', 'VSEP ($\mu \pm \sigma$)', 'Ours ($\mu \pm \sigma$)'],prop={'size': 15}, loc="center right" )

# NUMBER stats

print("FUEL stats", *get_target_info_stat(data_fuel, threshold=lambda_min, norm_factor=MAX_ENTROPY))
print("FUEL_f0_i10 stats", *get_target_info_stat(data_fuel_f0_i10, threshold=lambda_min, norm_factor=MAX_ENTROPY)) 
print("VSEP stats" , *get_target_info_stat(data_vsep, threshold=lambda_min, norm_factor=MAX_ENTROPY) )
print("SS_AP stats" , *get_target_info_stat(data_ss_ap, threshold=lambda_min, norm_factor=MAX_ENTROPY)) 

plt.show()



"""
RESULTS

Earthquake:
FUEL 0\%  &  -              &   -               &  -              & $106.0 \pm 9.8$   &  $155.3 \pm 48.4$ & $0.952 \pm 0.014$ \\
VSEP 10\%  &  $47.7 \pm 0.0$ &   $205.9 \pm 0.0$ &  NA             & $212.7 \pm 25.4$  &  NA                &  $0.739 \pm 0.058$ \\ 
OURS 100\%  &  $23.8 \pm 3.2$ &   $49.6 \pm 8.1 $ &  85.2 \pm 27.8  & $143.4 \pm 8.7$   &  $211.4 \pm 47.1$  & $0.990 \pm  0.002$ \\ 

Cave:
FUEL 0\%  &    -  &  -  &  -  &   $108.8 \pm 4.8$  &  $66.5 \pm 12.4$  &  $0.0925 \pm 0.002$\\
VSEP 0\%  &   -  &  -  &  NA  &   $272.3 \pm 84.1$  &  $NA $  & $0.0852 \pm 0.007$ \\
Ours 60\%  & $45.6 \pm 3.9$ & $96.1 \pm 6.5$  & $125.0 \pm 16.7$ & $153.3 \pm 15.9$  &  $155.0 \pm 25.0$  & $0.0959 \pm 0.001$ \\

"""