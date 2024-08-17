import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np
from utils import get_target_info

def plot_tivs(ax, data, j, threshold=0.02, color="red", world="earthquake", max_bars=3):
    tivs = np.array([np.sum(d[:, 6:]>threshold, axis=0) for d in data])
    # print(data[0])
    if world =="earthquake":
        # labels= ["human", "plant", "table", "carpet", "dog", "wall", "blood", "rubble", "flashlight"]
        labels= ["plant", "carpet", "table", "rubble", "dog", "blood", "human"]
        label_idx = [1, 3, 2, 7, 4, 6, 0]
    elif world =="cave":
        labels= ["rope","flashlight","radio","dog","blood","human"]

    for i in range(len(labels)):
        plot_error_strip(ax, tivs[:,label_idx[i]], j + (max_bars+1)*i+1, color)
    ax.set_yticks(range(2,(max_bars+1)*len(labels), (max_bars+1)))
    ax.set_yticklabels(labels,fontsize=15,color="black")

def plot_semantic_on_wif(df, ax, label="human", color = "red", threshold = 0.02):
    target_info = get_target_info(df)
    if target_info:
        idx, time, _, _ = target_info
        # print(df["wif"][idx.index].values)
        ax.plot(df["time"][idx.index].values, df["wif"][idx.index].values, color=color, marker="o", markersize=6, label="Target detections")
        ax.vlines(df["time"][idx.index].values, 0, df["wif"][idx.index].values, color="darkgray")
     
def plot_error_strip(ax, data, y_pos, color):
    mean = np.mean(data)
    std_dev = np.std(data)
    
    # Define the box limits
    box_left = mean - std_dev
    box_right = mean + std_dev
    vsize = 0.4
    # Plot the 'box' which is actually a rectangle from mean-std_dev to mean+std_dev
    ax.fill_betweenx([y_pos - vsize, y_pos + vsize], box_left, box_right, color=color)
    
    # Plot the mean as a vertical line
    ax.plot([mean, mean], [y_pos - vsize, y_pos + vsize], color="gray", linestyle='-', linewidth=2)

def plot_info_gain(df, ax, color, label):
    # Plot information gain
    # ax1.set_title("Weighted Information gain vs. Time")
    ax.set_xlabel('Time(s)', fontsize=17, color="black")
    ax.set_ylabel('Information gain', fontsize=17,color="black")

    times = (df["timestamp"] - df["timestamp"][0]).values
    ax.plot(times[::3], df["wif"].values[::3], linewidth=3, color=color, label=label)

    ax.tick_params(axis='x', labelsize=15)
    ax.tick_params(axis='y', labelsize=15)


def plot_mean_std(mean, std, ax, color, label):
    ax.plot(range(1, len(mean) + 1), mean, color=color,  linewidth=2)
    ax.plot(len(mean), mean[-1], color=color, markersize=5, marker="o", label=label+" $I^w_{t_f}$",linestyle='None')
    if color=="goldenrod":
        std_color = "lemonchiffon"
    elif color=="yellow":
        std_color = "lemonchiffon"
    elif color=="red":
        std_color = "pink"
    elif color=="darkorange":
        std_color = "peachpuff"
    elif color=="seagreen":
        std_color = "turquoise"
    elif color=="darkviolet":
        std_color = "lavender"
    elif color=="blue":
        std_color = "lightskyblue"
    else:
        std_color = "light"+color
    ax.fill_between(range(1, len(mean) + 1), mean - std, mean + std, color=std_color, alpha=0.5, label= label + ' ($\mu \pm \sigma$)')

