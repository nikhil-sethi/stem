import numpy as np
import os
import pandas as pd

MAX_ENTROPY = 1208686.75  # earthquake: 115703.375, mine:  1208686.75

def process_directory(directory):
    data = []
    minsize = 10000
    for filename in os.listdir(directory):
        if filename.endswith(".csv"):
            file_path = os.path.join(directory, filename)
            df = pd.read_csv(file_path)
            data.append(df.to_numpy())
    return data

def tolerant_mean(arrs):
    lens = [len(i) for i in arrs]
    arr = np.ma.empty((np.max(lens),len(arrs)))
    arr.mask = True
    for idx, l in enumerate(arrs):
        arr[:len(l),idx] = l
    return arr.mean(axis = -1), arr.std(axis=-1)


def get_target_info_stat(data, threshold=0.02, idx=6):
    """Gives the mu and std of path_length, time, and energy"""
    info = []
    # info  = np.ma.masked()
    for eps in data:
        arr = []
        try:
            target_idx = np.argwhere(eps[:,idx]>threshold)[0][0]
            arr.extend(eps[target_idx, 2:5]) # target: path length, time, energy
        except IndexError:
            arr.extend(np.ones(3)*-1)   

        arr.extend(eps[-1, 3:5]) # total: time, energy
        arr.append(eps[-1, 5]/MAX_ENTROPY) # Relative WIG at episode end

        info.append(arr)

    info = np.ma.masked_equal(info, -1)
    success = np.sum(~info.mask[:,0])/info.shape[0]
    return np.mean(info, axis=0), np.std(info, axis=0), success

def get_target_info(df, threshold=0.02, label="human"):
    # area_threshold = 0.02   # can also include temporal consistency

    # find target timestamp
    success_idx = (df["timestamp"][df[label]>=threshold])  # first instance when target area crosses threshold

    if not success_idx.empty:
        success_time = df["timestamp"][success_idx.index] - df["timestamp"][0]
        success_path_length = df["path_length"]
        success_energy = df["energy"]
        return success_idx, success_path_length, success_time, success_energy

def get_mean_std(data, idx):
    mean, std = tolerant_mean([episode[:,idx] for episode in data])
    return mean, std


"""


 






"""