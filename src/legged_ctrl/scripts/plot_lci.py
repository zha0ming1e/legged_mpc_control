"""set_model_state

Usage:
    set_model_state.py --filepath=<path>
"""
# references 
# plot two pandas frame https://stackoverflow.com/questions/46011940/how-to-plot-two-pandas-time-series-on-same-plot-with-legends-and-secondary-y-axi

import bagpy 
from bagpy import bagreader
import numpy as np
import matplotlib
import matplotlib.pyplot as plt 
import pandas as pd
from docopt import docopt

if __name__ == "__main__":
    args = docopt(__doc__)
    filepath = args["--filepath"]

    ## Read the relevant topics and data from the bag files 
    b = bagreader(filepath)

    ## Odometry and Odometry desired 
    fig, axes = plt.subplots(nrows=3, ncols=1)
    axes[0].set_title("position and desired position")
    odom_csv = b.message_by_topic("/a1_debug/odom")
    odom_d_csv = b.message_by_topic("/a1_debug/odom_d")
    odom_df = pd.read_csv(odom_csv)
    odom_d_df = pd.read_csv(odom_d_csv)
    odom_df.loc[:, ["pose.pose.position.x", "Time"]].plot(x="Time", ax=axes[0])
    odom_d_df.loc[:, ["pose.pose.position.x", "Time"]].plot(x="Time", ax=axes[0]) 
    axes[0].legend(["x", "x_d"])
    odom_df.loc[:, ["pose.pose.position.y", "Time"]].plot(x="Time", ax=axes[1])
    odom_d_df.loc[:, ["pose.pose.position.y", "Time"]].plot(x="Time", ax=axes[1]) 
    axes[1].legend(["y", "y_d"])
    odom_df.loc[:, ["pose.pose.position.z", "Time"]].plot(x="Time", ax=axes[2])
    odom_d_df.loc[:, ["pose.pose.position.z", "Time"]].plot(x="Time", ax=axes[2]) 
    axes[2].legend(["z", "z_d"])

    ## Euler and Euler d 
    fig, axes = plt.subplots(nrows=3, ncols=1)
    euler_csv = b.message_by_topic("/a1_debug/euler")
    euler_d_csv = b.message_by_topic("/a1_debug/euler_d")
    euler_df = pd.read_csv(euler_csv)
    euler_d_df = pd.read_csv(euler_d_csv)
    axes[0].set_title("Euler angles")
    euler_df.loc[:, ["vector.x", "Time"]].plot(x="Time", ax=axes[0])
    euler_d_df.loc[:, ["vector.x", "Time"]].plot(x="Time", ax=axes[0])
    axes[0].legend(["roll", "roll_d"])
    euler_df.loc[:, ["vector.y", "Time"]].plot(x="Time", ax=axes[1])
    euler_d_df.loc[:, ["vector.y", "Time"]].plot(x="Time", ax=axes[1])
    axes[1].legend(["pitch", "pitch_d"])
    euler_df.loc[:, ["vector.z", "Time"]].plot(x="Time", ax=axes[2])
    euler_d_df.loc[:, ["vector.z", "Time"]].plot(x="Time", ax=axes[2])
    axes[2].legend(["yaw", "yaw_d"])
    plt.xlabel("Time")
    plt.ylabel("radians")

    ## stance foor forces 
    fig, axes = plt.subplots(nrows=3, ncols=1)
    axes[0].set_title("stance foot forces")
    stance_csv = b.message_by_topic("/a1_debug/stance_foot_forces")
    stance_df = pd.read_csv(stance_csv)
    stance_df.loc[:, ["effort_2", "effort_5", "effort_8", "effort_11", "Time"]].plot(x="Time", ax=axes[0]) 
    axes[0].legend(["FL_z", "FR_z", "RL_z", "RR_z"])
    stance_df.loc[:, ["effort_0", "effort_3", "effort_6", "effort_9", "Time"]].plot(x="Time", ax=axes[1]) 
    axes[1].legend(["FL_x", "FR_x", "RL_x", "RR_x"])
    stance_df.loc[:, ["effort_1", "effort_4", "effort_7", "effort_10", "Time"]].plot(x="Time", ax=axes[2]) 
    axes[2].legend(["FL_y", "FR_y", "RL_y", "RR_y"])


    ## swing foot forces 
    fig, axes = plt.subplots(nrows=3, ncols=1)
    axes[0].set_title("swing foot forces")
    swing_csv = b.message_by_topic("/a1_debug/swing_foot_forces")
    swing_df = pd.read_csv(swing_csv)
    swing_df.loc[:, ["effort_2", "effort_5", "effort_8", "effort_11", "Time"]].plot(x="Time", ax=axes[0]) 
    axes[0].legend(["FL_z", "FR_z", "RL_z", "RR_z"])
    swing_df.loc[:, ["effort_0", "effort_3", "effort_6", "effort_9", "Time"]].plot(x="Time", ax=axes[1]) 
    axes[1].legend(["FL_x", "FR_x", "RL_x", "RR_x"])
    swing_df.loc[:, ["effort_1", "effort_4", "effort_7", "effort_10", "Time"]].plot(x="Time", ax=axes[2]) 
    axes[2].legend(["FL_y", "FR_y", "RL_y", "RR_y"])

    ## Foot positions 
    prefix = "/a1_debug"
    foot_names = ["/FL", "/FR", "/RL", "/RR"]
    fig, axes = plt.subplots(nrows=4, ncols=1)
    axes[0].set_title("Foot positions in body frame")
    for i, foot_name in enumerate(foot_names):
        pose_df = pd.read_csv(b.message_by_topic(prefix+ foot_name + "/pose"))
        target_df = pd.read_csv(b.message_by_topic(prefix + foot_name + "/target"))
        pose_df.loc[:, ["pose.position.x", "pose.position.y", "pose.position.z", "Time"]].plot(x="Time", ax=axes[i])
        target_df.loc[:, ["pose.position.x", "pose.position.y", "pose.position.z", "Time"]].plot(x="Time", ax=axes[i])
    
    ## Contact forces 
    contact_f_csv = b.message_by_topic("/a1_debug/contact_forces")
    contact_df = pd.read_csv(contact_f_csv)
    contact_df.loc[:, ["effort_0", "effort_1", "effort_2", "effort_3", "Time"]].plot(x="Time") 
    plt.legend(["FL", "FR", "RL", "RR"])
    
    plt.show()