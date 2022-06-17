#!/usr/bin/env python3

import hebi
from time import sleep, time
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation as R

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "HEBI"
module_name = "mobileIO"

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

# Live Visualization
# Starts logging in the background. Note that logging can be enabled at any time, and that it does not negatively
# affect the performance of your running programs.
group.start_log('dir', 'logs', mkdirs=True)

print('Visualizing 6-DoF Pose estimate from the mobile device.')
print('Move it around to make the feedback interesting...')


plt.ion()
f = plt.figure()


duration = 10.0
start_time = time()
end_time = start_time + duration
current_time = start_time
fbk = hebi.GroupFeedback(group.size)

x = np.array([
    [0, 1],
    [0, 0],
    [0, 0]
])

y = np.array([
    [0, 0],
    [0, 1],
    [0, 0]
])

z = np.array([
    [0, 0],
    [0, 0],
    [0, 1]
])

while current_time < end_time:
    current_time = time()
    fbk = group.get_next_feedback(reuse_fbk=fbk)
    if fbk is None:
        print("Could not get feedback")
        continue

    orient = fbk[0].ar_orientation
    # reorder w,x,y,z to x,y,z,w
    orient = [*orient[1:], orient[0]]
    pos = fbk[0].ar_position
    # Scale pos for eye candy
    pos = pos * 2

    r = R.from_quat(orient)
    rot_mat = r.as_matrix()

    plt.clf()
    ax = plt.axes(projection="3d")
    ax.set_xlim3d(-1.5, 1.5)
    ax.set_ylim3d(-1.5, 1.5)
    ax.set_zlim3d(-1.5, 1.5)

    r_x = rot_mat @ x
    r_y = rot_mat @ y
    r_z = rot_mat @ z

    x_lineX = r_x[0, :] + (pos[0], pos[0])
    x_lineY = r_x[1, :] + (pos[1], pos[1])
    x_lineZ = r_x[2, :] + (pos[2], pos[2])

    y_lineX = r_y[0, :] + (pos[0], pos[0])
    y_lineY = r_y[1, :] + (pos[1], pos[1])
    y_lineZ = r_y[2, :] + (pos[2], pos[2])

    z_lineX = r_z[0, :] + (pos[0], pos[0])
    z_lineY = r_z[1, :] + (pos[1], pos[1])
    z_lineZ = r_z[2, :] + (pos[2], pos[2])

    ax.plot3D(x_lineX, x_lineY, x_lineZ)
    ax.plot3D(y_lineX, y_lineY, y_lineZ)
    ax.plot3D(z_lineX, z_lineY, z_lineZ)

    plt.pause(0.001)

print('All done!')

log_file = group.stop_log()

if log_file is not None:
    hebi.util.plot_logs(log_file, 'position', figure_spec=101)
