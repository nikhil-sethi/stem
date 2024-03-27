import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

odom_df = pd.read_csv("odom.csv")
cmd_df = pd.read_csv("cmd.csv")
# print(vel)

time_secs = (odom_df["%time"] -  odom_df["%time"][0])/1e9


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
    siny_cosp = 2 * (arr[:,3] * arr[:,2] );
    cosy_cosp = 1 - 2 * (arr[:,2] * arr[:,2]);
    yaws = np.arctan2(siny_cosp, cosy_cosp);
    # yaws[yaws<0] += 2*np.pi 
    yaws = np.unwrap(yaws)
    return yaws

cmd_idx = find_sync_values(odom_df["%time"]/1e9, cmd_df["%time"]/1e9, 0.025)
cmd_df = cmd_df.loc[cmd_idx]

print(len(cmd_idx))
fig, (ax_pos, ax_yaw) = plt.subplots(2)

## position setpoint and actual 
ax_pos.set_prop_cycle(color=['red','green','blue'])
ax_pos.plot(time_secs.to_numpy(), odom_df[["field.pose.pose.position.x", "field.pose.pose.position.y", "field.pose.pose.position.z"]].to_numpy(), label=['x', 'y', 'z'])
ax_pos.plot(time_secs.to_numpy(), cmd_df[["field.pose.position.x", "field.pose.position.y", "field.pose.position.z"]].to_numpy()[:len(time_secs)], linestyle="--")

# yaw setpoint vs actual
odom_yaws  = quat_to_yaws(odom_df[["field.pose.pose.orientation.x", "field.pose.pose.orientation.y", "field.pose.pose.orientation.z", "field.pose.pose.orientation.w"]].to_numpy())
cmd_yaws = quat_to_yaws(cmd_df[["field.pose.orientation.x", "field.pose.orientation.y", "field.pose.orientation.z", "field.pose.orientation.w"]].to_numpy())
ax_yaw.set_xlabel("Time")
ax_yaw.set_ylabel("Position")


ax_yaw.plot(time_secs.to_numpy(), odom_yaws, color="red")
ax_yaw.plot(time_secs.to_numpy(), cmd_yaws, linestyle="--", color="red")
ax_yaw.set_xlabel("Time")
ax_yaw.set_ylabel("Yaw")
# fig.set_title("Setpoints vs. Actual")
plt.legend()
# ax[0].scatter(x='Time', y='linear.x', data=vel)
plt.show()