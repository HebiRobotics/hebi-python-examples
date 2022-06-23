#!/usr/bin/env python3

import hebi
from time import sleep, time
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
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

print('Visualizing 3-DoF orientation estimate from the mobile device.')
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

    try:
        r = R.from_quat(orient)
        rot_mat = r.as_matrix()
    except:
        rot_mat = np.eye(3)

    plt.clf()
    ax = plt.axes(projection="3d")
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)

    r_x = rot_mat @ x
    r_y = rot_mat @ y
    r_z = rot_mat @ z

    ax.plot3D(r_x[0, :], r_x[1, :], r_x[2, :])
    ax.plot3D(r_y[0, :], r_y[1, :], r_y[2, :])
    ax.plot3D(r_z[0, :], r_z[1, :], r_z[2, :])

    plt.pause(0.001)

print('All done!')
