#!/usr/bin/env python3

import hebi
import numpy as np
import os
import sys


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


from util.math_utils import get_grav_comp_efforts
from util.arm import setup_arm_params
from time import sleep, time
from util import math_utils
from hebi.util import create_mobile_io

run_mode = "points"

first_run = True

# Mobile device setup
phone_family = 'HEBI'
phone_name   = "mobileIO"

lookup = hebi.Lookup()
sleep(2)

print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.set_button_mode(1, 'momentary')
m.set_button_mode(2, 'momentary')
m.set_button_mode(3, 'momentary')
m.update()

quit_demo_button = 8
abort_flag = False
lookup = hebi.Lookup()
sleep(2)

# Arm Setup
arm_family = 'Example Arm'
arm_name   = '6-DoF'
has_gas_spring = False # If you attach a gas spring to the shoulder for extra payload, set this to True.

group, kin, params = setup_arm_params(arm_name, arm_family, has_gas_spring, lookup=lookup)

fbk = hebi.GroupFeedback(group.size)
effort_offset = params.effort_offset
gravity_vec = params.gravity_vec
local_dir = params.local_dir

arm_dof_count = kin.dof_count

print('B1-3 - select different points to move the arm to.')
print('B6 - enables grav comp mode.')
print('B8 - Quits the demo.')
                           
cmd = hebi.GroupCommand(group.size)

# Point coordinates
point_1 = np.asarray([np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, np.pi/4, 0])
point_2 = np.asarray([-np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, 3*np.pi/4, 0])
point_3 = np.asarray([0, 0, 0, 0, 0, 0])

end_velocities = np.zeros(arm_dof_count)
end_accels = np.zeros(arm_dof_count)

# Zero the times
t = 0
duration = 0

time_vector = [0, 5]
start = time()

pos_cmd = np.zeros(group.size)
vel_cmd = np.zeros(group.size)
acc_cmd = np.zeros(group.size)


def get_trajectory(curr_pos, curr_vel, curr_acc, point):
  """
  Helper function to create a trajectory
  """
  waypoints = np.empty((group.size, 2))
  waypoints[:, 0] = curr_pos
  waypoints[:, 1] = point

  velocities = np.empty((group.size, 2))
  velocities[:, 0] = curr_vel
  velocities[:, 1] = end_velocities

  accels = np.empty((group.size, 2))
  accels[:, 0] = curr_acc
  accels[:, 1] = end_accels

  trajectory = hebi.trajectory.create_trajectory(time_vector, waypoints, velocities, accels)
  return trajectory


while not abort_flag:
  # Update MobileIO state
  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  # Update arm state
  group.get_next_feedback(reuse_fbk=fbk)

  # Check for quit
  if m.get_button_state(quit_demo_button):
    # Set led red and quit
    m.set_led_color("red")
    abort_flag = False
    break

  # On first run go to point 1
  if first_run:
    run_mode = "points"
    first_run = False
    trajectory = get_trajectory(fbk.position, end_velocities, end_accels, point_1)

    duration = trajectory.duration
    start = time()
    t = time() - start

  # If button 1 pressed, create trajectory from current position to point 1
  elif m.get_button_diff(1) == 3: # "ToOn"
    run_mode = "points"
    trajectory = get_trajectory(pos_cmd, vel_cmd, acc_cmd, point_1)

    duration = trajectory.duration
    start = time()
    t = time() - start

  # If button 2 pressed, create trajectory from current position to point 2
  elif m.get_button_diff(2) == 3: # "ToOn"
    run_mode = "points"
    trajectory = get_trajectory(pos_cmd, vel_cmd, acc_cmd, point_2)

    duration = trajectory.duration
    start = time()
    t = time() - start

  # If button 3 pressed, create trajectory from current position to point 3
  elif m.get_button_diff(3) == 3: # "ToOn"
    run_mode = "points"
    trajectory = get_trajectory(pos_cmd, vel_cmd, acc_cmd, point_3)

    duration = trajectory.duration
    start = time()
    t = time() - start

  # If button 5 pressed switch to grav comp mode    
  elif m.get_button_diff(5) == 3: # "ToOn"
    run_mode = "grav comp"

  if run_mode == "points":
    # Set led to green for point move mode
    m.set_led_color("green")
    # Move to point
    if t < duration:
      t = time() - start

      # Get new commands from the trajectory
      pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)

      # Calculate commanded efforts to assist with tracking the trajectory.
      # Gravity Compensation uses knowledge of the arm's kinematics and mass to
      # compensate for the weight of the arm. Dynamic Compensation uses the
      # kinematics and mass to compensate for the commanded accelerations of the arm.
      eff_cmd = math_utils.get_grav_comp_efforts(kin, fbk.position, [0, 0, 1])
      # NOTE: dynamic compensation effort computation has not yet been added to the APIs

      # Fill in the command and send commands to the arm
      cmd.position = pos_cmd
      cmd.velocity = vel_cmd
      cmd.effort = eff_cmd
      group.send_command(cmd)
    else:
      cmd.position = pos_cmd
      group.send_command(cmd)

  elif run_mode == "grav comp":
    # Set led to blue for grav comp mode
    m.set_led_color("blue")
    # Grav comp mode
    # Update gravity vector the base module of the arm
    params.update_gravity(fbk)
    gravity_vec = params.gravity_vec

    # Calculate required torques to negate gravity at current position
    cmd.effort = get_grav_comp_efforts(kin, fbk.position, -gravity_vec) + effort_offset
    nan_array = np.empty(group.size)
    nan_array[:] = np.nan
    cmd.position = nan_array
    cmd.velocity = nan_array

    # Update these so when creating next trajectory in point move mode they are current
    pos_cmd = fbk.position
    vel_cmd = fbk.velocity
    acc_cmd = end_accels
    # Send to robot
    group.send_command(cmd)
