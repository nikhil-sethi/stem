import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

bag_file_name = "/root/thesis_ws/src/thesis/sw/bringup/data/bags/2024-03-27-21-02-50.bag"
bag = bagreader(bag_file_name)
odom_data = bag.message_by_topic("/mavros/local_position/odom")
cmd_data = bag.message_by_topic("/mavros/setpoint_position/local")

odom_df = pd.read_csv(odom_data)
cmd_df = pd.read_csv(cmd_data)

odom_df = odom_df.loc[::2] # filter out some repeated very small dt timestamps. Helps in velocity filtering
# print(vel)

time_secs = (odom_df["Time"] -  odom_df["Time"][0])   # time in seconds from when odometry started 


def find_nearest(array, value):
    """Find the index of the nearest value in the array."""
    idx = np.abs(array - value).argmin()
    return idx

def find_sync_values(array_a, array_b, tolerance):
    """Find values in array B that sync approximately closely with values in array A."""
    synced_values = []
    for value_a in array_a:
        idx = find_nearest(array_b, value_a)
        if np.abs(array_b[idx] - value_a) <= tolerance:
            synced_values.append(idx)
    return np.array(synced_values)



def quat_to_yaws(arr):
    """
    Converts quaternion rotation abouot z axis to y axis
    """
    siny_cosp = 2 * (arr[:,3] * arr[:,2] );
    cosy_cosp = 1 - 2 * (arr[:,2] * arr[:,2]);
    yaws = np.arctan2(siny_cosp, cosy_cosp);
    # yaws[yaws<0] += 2*np.pi 
    yaws = np.unwrap(yaws)
    return yaws

def yd_from_yaws(yaws, times, window):
    # Calculate time differences
    time_diff = np.diff(times)

    # Calculate yaw differences
    yaw_diff = np.diff(yaws)

    # Calculate angular velocity (angular change per unit time)
    angular_velocity = yaw_diff / time_diff

    # Apply a moving average filter to smooth the angular velocity
    # angular_velocity_smoothed = np.convolve(angular_velocity, np.ones(window)/window, mode='valid')

    return angular_velocity


# the cmd commands are recorded at a different frequency. so this is just naively matching the timestamp to odometry within a certain accuracy
cmd_idx = find_sync_values(odom_df["Time"], cmd_df["Time"], 0.025)
cmd_df = cmd_df.loc[cmd_idx]

# print(len(cmd_idx))
fig, (ax_pos, ax_yaw) = plt.subplots(2)

## position setpoint and actual 
ax_pos.set_prop_cycle(color=['red','magenta','blue'])
ax_pos.plot(time_secs.to_numpy(), odom_df[["pose.pose.position.x", "pose.pose.position.y", "pose.pose.position.z"]].to_numpy(), label=['x', 'y', 'z'])
ax_pos.plot(time_secs.to_numpy(), cmd_df[["pose.position.x", "pose.position.y", "pose.position.z"]].to_numpy()[:len(time_secs)], linestyle="--")

ax_pos.set_xlabel("Time")
ax_pos.set_ylabel("Position")


# yaw setpoint vs actual
odom_yaws  = quat_to_yaws(odom_df[["pose.pose.orientation.x", "pose.pose.orientation.y", "pose.pose.orientation.z", "pose.pose.orientation.w"]].to_numpy())
cmd_yaws = quat_to_yaws(cmd_df[["pose.orientation.x", "pose.orientation.y", "pose.orientation.z", "pose.orientation.w"]].to_numpy())


ax_yaw.plot(time_secs.to_numpy(), odom_yaws, 'r.-')
ax_yaw.plot(time_secs.to_numpy(), cmd_yaws, linestyle="--", color="red")

# angular velocity vs time

# ax_yaw.plot(time_secs.to_numpy()[window_size // 2: -window_size // 2], odom_yds, color="blue")
ax_yaw.plot(time_secs.to_numpy(), odom_df[["twist.twist.angular.x", "twist.twist.angular.y", "twist.twist.angular.z"]].to_numpy(), color="blue")


ax_yaw.set_xlabel("Time")
ax_yaw.set_ylabel("Yaw")
# fig.set_title("Setpoints vs. Actual")
plt.legend()
# ax[0].scatter(x='Time', y='linear.x', data=vel)
plt.show()