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
module_name = "Mobile IO"

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

while current_time < end_time:
  current_time = time()
  fbk = group.get_next_feedback()
  orient = fbk[0].ar_orientation
  pos = fbk[0].ar_position
  # Scale pos for eye candy
  pos = pos*2

  r = R.from_quat(orient)
  rot_mat = r.as_matrix()

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

  plt.clf()
  ax = plt.axes(projection="3d")
  ax.set_xlim3d(-1.5, 1.5)
  ax.set_ylim3d(-1.5, 1.5)
  ax.set_zlim3d(-1.5, 1.5)

  x_lineX = (rot_mat @ x)[0, :] + (pos[0], pos[0])
  x_lineY = (rot_mat @ x)[1, :] + (pos[1], pos[1])
  x_lineZ = (rot_mat @ x)[2, :] + (pos[2], pos[2])

  y_lineX = (rot_mat @ y)[0, :] + (pos[0], pos[0])
  y_lineY = (rot_mat @ y)[1, :] + (pos[1], pos[1])
  y_lineZ = (rot_mat @ y)[2, :] + (pos[2], pos[2])

  z_lineX = (rot_mat @ z)[0, :] + (pos[0], pos[0])
  z_lineY = (rot_mat @ z)[1, :] + (pos[1], pos[1])
  z_lineZ = (rot_mat @ z)[2, :] + (pos[2], pos[2])

  ax.plot3D(x_lineX, x_lineY, x_lineZ)
  ax.plot3D(y_lineX, y_lineY, y_lineZ)
  ax.plot3D(z_lineX, z_lineY, z_lineZ)

  plt.pause(0.01)
print('All done!')