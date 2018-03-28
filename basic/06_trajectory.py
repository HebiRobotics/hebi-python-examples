#!/usr/bin/env python3

from hebi import *
from math import pi
from time import sleep
import numpy as np

# Get a group
lookup = Lookup()
sleep(2)

group = lookup.get_group_from_names(['family'], ['shoulder', 'elbow'])

if not group:
  print('Group not found!')
  exit(1)

num_joints = group.size
group.command_lifetime = 100

if not group.send_feedback_request():
  print('Could not send feedback request.')
  exit(1)

group_fbk = group.get_next_feedback()
if not group_fbk:
  print('Could not get feedback.')
  exit(1)

# ------------------------------------------------------------------------------
# Position, velocity, and acceleration waypoints. Each column is a separate
# waypoint - each row is a different joint
# ------------------------------------------------------------------------------

# pos[0], 0, pi/2, 0    , 0
# pos[1], 0, 0   , -pi/2, 0
positions = np.zeros((num_joints, 5))
positions[0:2, 0] = group_fbk.position[0:2].T
positions[0, 2] = pi/2
positions[1, 3] = -pi/2

# 0, nan, nan, nan, 0
# 0, nan, nan, nan, 0
velocities = np.zeros((num_joints, 5))
velocities[0:2, 1:4] = np.nan

# 0, nan, nan, nan, 0
# 0, nan, nan, nan, 0
accelerations = velocities

# 0, 5, 10, 15, 20
time = np.linspace(0, 20, 5)

trajectory = trajectory.create_trajectory(time, positions, velocities, accelerations)

# Follow the trajectory
group_cmd = GroupCommand(num_joints)
period = 0.01
duration = trajectory.duration

t = 0.0
while(t < duration):
  pos_cmd, vel_cmd, eff_cmd = trajectory.get_state(t)
  group_cmd.position = pos_cmd
  group_cmd.velocity = vel_cmd
  group.send_command(group_cmd)

  t = t + period
  sleep(period)
