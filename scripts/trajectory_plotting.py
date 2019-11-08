import matplotlib.pyplot as plt
import numpy as np

TRAJECTORIES = ["/home/hejia/ros/ros_ws/devel/bin/cached_traj.txt", "/home/hejia/ros/ros_ws/devel/bin/cached_traj_wield.txt"]

fig, axes = plt.subplots(2, 6)

for i in range(2):
    traj = [[], [], [], [], [], []]
    with open(TRAJECTORIES[i], 'r') as f:
        lines = f.readlines()
        for line in lines:
            tokens = line.split(' ')[:-1]
            for k, token in enumerate(tokens):
                traj[k].append(float(token))
    for j in range(6):
        axes[i, j].plot(range(len(traj[j])), np.array(traj[j]))

plt.show()

