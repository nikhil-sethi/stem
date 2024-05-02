import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import itertools

def process_directory(directory):
    """
    Process CSV files in a directory and return mean, std, and information gain of entropy values.
    """
    min_entropies = []
    info_gains = []
    for filename in os.listdir(directory):
        if filename.endswith(".csv"):
            file_path = os.path.join(directory, filename)
            df = pd.read_csv(file_path)
            min_entropies.append(np.min(df['entropy '].values))  # Corrected key name
            info_gains.append(np.cumsum(-np.diff(df['entropy '].values)))
            
    max_rows = max(len(info_gain) for info_gain in info_gains)
    for i in range(len(info_gains)):
        diff = max_rows - len(info_gains[i])
        if diff > 0:
            # min_entropies[i] = np.append(min_entropies[i], [np.nan] * diff)
            info_gains[i] = np.append(info_gains[i], [np.nan] * diff)
    # mean_entropy = np.nanmean(entropy_values, axis=0)
    # std_entropy = np.nanstd(entropy_values, axis=0)
    # Calculate information gain
    # info_gain = np.cumsum(-np.diff(mean_entropy))  # Added negative sign
    # print(info_gains)
    info_gains = np.array(info_gains)
    mean_info_gains = np.nanmean(info_gains, axis=0)
    std_info_gains = np.nanstd(info_gains, axis=0)

    return min_entropies, mean_info_gains, std_info_gains

def stack_padding(l):
    return np.column_stack((itertools.zip_longest(*l, fillvalue=0)))



# Process 'logs/exploration' directory
exploration_min_entropies, exploration_info_gain, exploration_info_gain_std = process_directory('logs/exploration/')

# Process 'logs/target_search' directory
target_search_min_entropies, target_search_info_gain, target_search_info_gain_std = process_directory('logs/target_search/')

labels = ['Exporation', 'Target search']
# arg = np.argmax(len(exploration_min_entropies), len(target_search_min_entropies))
# fills = abs(len(exploration_min_entropies) - len(target_search_min_entropies))



min_entropies_all= [exploration_min_entropies, target_search_min_entropies]

# min_entropies_all[np.argmax(min_entropies_all)] = min_entropies_all[np.argmax(min_entropies_all)][:len(min_entropies_all[np.argmin(min_entropies_all)])]
# min_entropies_all = np.column_stack([*min_entropies_all])
min_entropies_all = stack_padding(min_entropies_all).T
print(min_entropies_all.shape)

# Plotting for entropy
fig1, ax1 = plt.subplots(figsize=(10, 6))

# Plot 'exploration' entropy data
color = 'tab:blue'
ax1.set_xlabel('Time')
ax1.set_ylabel('Entropy')
# ax1.plot(range(1, len(exploration_mean) + 1), exploration_mean, color=color, label='Exploration Mean')
# ax1.fill_between(range(1, len(exploration_mean) + 1), exploration_mean - exploration_std, exploration_mean + exploration_std, color='lightblue', alpha=0.5, label='Exploration Std Dev')

# ax1.plot(1, exploration_mean, color=color, label='Exploration Mean', markerstyle="o")
bplot1 = ax1.boxplot(min_entropies_all,
                     vert=True,  # vertical box alignment
                     patch_artist=True,  # fill with color
                     labels=labels)  # will be used to label x-ticks

# ax1.fill_between(range(1, len(exploration_mean) + 1), exploration_mean - exploration_std, exploration_mean + exploration_std, color='lightblue', alpha=0.5, label='Exploration Std Dev')

# ax1.tick_params(axis='y', labelcolor=color)

# Plot 'target_search' entropy data
color = 'tab:orange'
# ax1.plot(range(1, len(target_search_mean) + 1), target_search_mean, color=color, label='Target Search Mean')
# ax1.plot(2, target_search_mean, color=color, label='Target Search Mean')
# ax1.errorbar(2, target_search_mean, target_search_std, linestyle='None', marker='o', capsize=3,color=color)

# ax1.fill_between(range(1, len(target_search_mean) + 1), target_search_mean - target_search_std, target_search_mean + target_search_std, color='orange', alpha=0.5, label='Target Search Std Dev')
# ax1.tick_params(axis='y', labelcolor=color)

plt.title('Min. Entropy')
# ax1.legend(loc='upper left')
fig1.tight_layout()

# # Process 'logs/exploration' directory for information gain
# _, _, exploration_info_gain = process_directory('logs/exploration/')
# # Process 'logs/target_search' directory for information gain
# _, _, target_search_info_gain = process_directory('logs/target_search/')

# Plotting for information gain
fig2, ax2 = plt.subplots(figsize=(10, 6))

# Plot 'exploration' information gain
color = 'tab:blue'
ax2.set_xlabel('Time')
ax2.set_ylabel('Information Gain')
ax2.plot(range(1, len(exploration_info_gain) + 1), exploration_info_gain, color=color, label='Exploration Info Gain')
ax2.fill_between(range(1, len(exploration_info_gain) + 1), exploration_info_gain - exploration_info_gain_std, exploration_info_gain + exploration_info_gain_std, color='lightblue', alpha=0.5, label='Exploration Info Gain Std Dev')
ax2.tick_params(axis='y', labelcolor=color)

# Plot 'target_search' information gain
color = 'tab:orange'
ax2.plot(range(1, len(target_search_info_gain) + 1), target_search_info_gain, color=color, label='Target Search Info Gain')
ax2.fill_between(range(1, len(target_search_info_gain) + 1), target_search_info_gain - target_search_info_gain_std, target_search_info_gain + target_search_info_gain_std, color='orange', alpha=0.5, label='Target Search Info Gain Std Dev')
ax2.tick_params(axis='y', labelcolor=color)

plt.title('Information Gain vs Time')
ax2.legend(loc='upper left')
fig2.tight_layout()

plt.show()
