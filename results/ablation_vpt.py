import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
plt.style.use('ggplot')
from plotting import plot_info_gain, plot_semantic_on_wif

def plot_wif_and_target(ax, color, world, label):
    df = pd.read_csv(f"/root/thesis_ws/src/thesis/results/data/{world}/{label}/{label}.csv")
    plot_info_gain(df, ax, color, label)
    plot_semantic_on_wif(df, ax, label="human")
    plot_semantic_on_wif(df, ax, label="dog", color="blue", threshold=0.02)
    # plot_semantic(df, ax, label="blood", color="lime")

if __name__=="__main__":
    fig1, ax1 = plt.subplots()
    fig1.tight_layout()
    # ax1.grid()
    plt.ticklabel_format(axis='both', style='sci', scilimits=(0,4))

    world = "earthquake"

    plot_wif_and_target(fig1, ax1, color="tab:blue", world=world, label="FUEL")

    plot_wif_and_target(fig1, ax1, color="tab:orange",world=world, label="SS")

    plot_wif_and_target(fig1, ax1, color="hotpink", world=world, label="SS+AP")


    h, l = ax1.get_legend_handles_labels()
    # manually decide sequence for now
    seq = [0,1,3, 2]

    ax1.legend(np.array(h)[seq], np.array(l)[seq] ,loc='upper left',prop={'size': 15})

    plt.show()
