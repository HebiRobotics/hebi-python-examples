#!/usr/bin/env python3

import hebi
from math import pi
from time import sleep, time
import numpy as np
from matplotlib import pyplot as plt


# Add the root folder of the repository to the search path for modules
import os, sys
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path = [root_path] + sys.path
from util import math_utils


def get_group():
  """
  Helper function to create a group from named modules, and set specified gains on the modules in that group.
  """
  families = ['Test Family']
  names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
  lookup = hebi.Lookup()
  sleep(2.0)
  group = lookup.get_group_from_names(families, names)
  if group is None:
    return None

  # Set gains
  gains_command = hebi.GroupCommand(group.size)
  try:
    gains_command.read_gains("gains/A-2085-06.xml")
  except Exception as e:
    print('Failed to read gains: {0}'.format(e))
    return None
  if not group.send_command_with_acknowledgement(gains_command):
    print('Failed to receive ack from group')
    return None

  return group


def execute_trajectory(group, model, trajectory, feedback):
  """
  Helper function to actually execute the trajectory on a group of modules
  """
  num_joints = group.size
  command = hebi.GroupCommand(num_joints)
  duration = trajectory.duration

  start = time()
  t = time() - start

  while t < duration:
    # Get feedback and update the timer
    group.get_next_feedback(reuse_fbk=feedback)
    t = time() - start

    # Get new commands from the trajectory
    pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)

    # Calculate commanded efforts to assist with tracking the trajectory.
    # Gravity Compensation uses knowledge of the arm's kinematics and mass to
    # compensate for the weight of the arm. Dynamic Compensation uses the
    # kinematics and mass to compensate for the commanded accelerations of the arm.
    eff_cmd = model.get_grav_comp_efforts(feedback.position, [0, 0, 1])
    # NOTE: dynamic compensation effort computation has not yet been added to the APIs

    # Fill in the command and send commands to the arm
    command.position = pos_cmd
    command.velocity = vel_cmd
    command.effort = eff_cmd
    group.send_command(command)


# Get group of modules and set gains.
group = get_group()
if group is None:
  print('Group not found! Check that the family and name of a module on the network')
  print('matches what is given in the source file.')
  exit(1)

try:
  model = hebi.robot_model.import_from_hrdf("hrdf/A-2085-06.hrdf")
except Exception as e:
  print('Could not load HRDF: {0}'.format(e))
  exit(1)

# Go to the XYZ positions at four corners of the box, and create a rotation matrix
# that has the end effector point straight forward.
xyz_targets = np.array([[0.20, 0.40, 0.40, 0.20], [0.30, 0.30, -0.30, -0.30], [0.10, 0.10, 0.10, 0.10]])
xyz_cols = xyz_targets.shape[1]
rotation_target = math_utils.rotate_y(pi/2)

# Convert these to joint angle waypoints using IK solutions for each of the xyz locations
# as well as the desired orientation of the end effector. Copy the initial waypoint at the end so we close the square.

# Choose an "elbow up" initial configuration for IK
elbow_up_angles = [0, pi / 4.0, pi / 2.0, pi / 4.0, -pi, pi / 2.0]

joint_targets = np.empty((group.size, xyz_cols + 1))
for col in range(xyz_cols):
  so3_objective = hebi.robot_model.endeffector_so3_objective(rotation_target)
  ee_position_objective = hebi.robot_model.endeffector_position_objective(xyz_targets[:, col])
  ik_res_angles = model.solve_inverse_kinematics(elbow_up_angles, so3_objective, ee_position_objective)
  joint_targets[:, col] = ik_res_angles
  elbow_up_angles = ik_res_angles # reset seed after each loop
joint_targets[:, xyz_cols] = joint_targets[:, 0]

# Set up feedback object, and start logging
feedback = hebi.GroupFeedback(group.size)
group.start_log("logs", mkdirs=True)

# Get a trajectory from the current position to the first corner of the box:
waypoints = np.empty((group.size, 2))
group.get_next_feedback(reuse_fbk=feedback)
waypoints[:, 0] = feedback.position
waypoints[:, 1] = joint_targets[:, 0]
time_vector = [0.0, 5.0]  # Seconds for the motion - do this slowly
trajectory = hebi.trajectory.create_trajectory(time_vector, waypoints)

# Call helper function to execute this motion on the robot
execute_trajectory(group, model, trajectory, feedback)

# Go to all 4 corners. Calculate new point-to-point trajectories one at a time.
time_vector[1] = 3.0  # seconds for the move - do this a little bit more quickly
for col in range(xyz_cols - 1):
  waypoints[:, 0] = joint_targets[:, col]
  waypoints[:, 1] = joint_targets[:, col + 1]
  trajectory = hebi.trajectory.create_trajectory(time_vector, waypoints)
  execute_trajectory(group, model, trajectory, feedback)

# Stop logging
log_file = group.stop_log()
if log_file is not None:
  hebi.util.plot_logs(log_file, 'position', figure_spec=101)
  hebi.util.plot_logs(log_file, 'velocity', figure_spec=102)
  hebi.util.plot_logs(log_file, 'effort', figure_spec=103)
