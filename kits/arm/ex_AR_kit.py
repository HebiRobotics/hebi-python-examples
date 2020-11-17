#!/usr/bin/env python3

import hebi
import numpy as np
import os
import threading
import copy
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api

# Arm setup
arm_family   = "Arm"
phone_name   = "mobileIO"
hrdf_file    = "hrdf/A-2085-06.hrdf"

lookup = hebi.Lookup()
sleep(2)

# Setup MobileIO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, arm_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.update()

# Setup arm components
arm = arm_api.create([arm_family],
                     names=['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
                     lookup=lookup,
                     hrdf_file=hrdf_file)

keep_running = True
pending_goal = False
run_mode = "startup"
goal = arm_api.Goal(arm.size)
home_pos = 1
control_mode = 2
quit_demo_button = 8

xyz_target_init = np.asarray([0.5, 0.0, 0.0])
ik_seed_pos = np.asarray([0.01, np.pi/3 , 2*np.pi/3, 5*np.pi/6, -np.pi/2, 0.01])
mobile_pos_offset = [0, 0, 0]


def get_mobile_state(quit_demo_button):
  """
  Mobile io function to be run in another thread to prevent main loop stalling on long feedbacks
  """
  global fbk_mobile
  global keep_running
  global run_mode
  global mobile_pos_offset
  global mobile_pos_init
  global mobile_ori_init

  m.update()

  m.set_led_color("yellow")
  while not m.get_button_diff(quit_demo_button) == 3: # "ToOn"

    if not m.update():
      print("Failed to get feedback from MobileIO")
      continue
  
    fbk_mobile = m.get_last_feedback()
    # Check for button presses and control state accordingly
    if m.get_button_diff(home_pos) == 2: # "ToOff"
      run_mode = "startup"
      m.set_led_color("yellow")
    if run_mode == "standby":
      m.set_led_color("green")
    if m.get_button_diff(control_mode) == 3 and run_mode == "standby": # "ToOn"
      m.set_led_color("blue")
      run_mode = "control"
      mobile_pos_init = copy.copy(fbk_mobile.ar_position)
      mobile_ori_init = copy.copy(fbk_mobile.ar_orientation)
    if run_mode == "control":
      m.set_led_color("black")
      mobile_pos_offset = fbk_mobile.ar_position - mobile_pos_init
            
  m.set_led_color("red")
  keep_running = False


# Start mobile io thread
t1 = threading.Thread(target=get_mobile_state, args=(quit_demo_button,), daemon=True)
t1.start()

print('Arm end-effector is now following the mobile device pose.')
print('The control interface has the following commands:')
print('  B1 - Reset/re-align poses.')
print('       This takes the arm to home and aligns with mobile device.')
print('  B8 - Quits the demo.')


# Main run loop
while keep_running:

  if not arm.update():
    print("Failed to update arm")
    continue

  if run_mode == "startup":
    # Move to starting pos
    joint_targets = arm.ik_target_xyz(ik_seed_pos, xyz_target_init)
    arm.set_goal(goal.clear().add_waypoint(t=3, position=joint_targets))
    run_mode = "moving to start pos"
  elif run_mode == "moving to start pos":
    # When at startup pos, switch to standby mode
    if arm.at_goal:
      run_mode = "standby"
  elif run_mode == "standby":
    # Wait for mobile io input while holding arm in place
    pass
  elif run_mode == "control":
    # Follow phone's motion in 3D space
    xyz_target_new = xyz_target_init + mobile_pos_offset
    # ori_target_new = mobile_ori_init
    
    print(mobile_ori_init)

    joint_targets = arm.ik_target_xyz(arm.last_feedback.position, xyz_target_new)
    # joint_targets = arm.ik_target_xyz_so3(arm.last_feedback.position, xyz_target_new, ori_target_new)
    arm.set_goal(goal.clear().add_waypoint(t=1.0, position=joint_targets.tolist()))

  arm.send()


# I pretty much just need to rewrite this example, my own way. Along with the other examples in this repo.
# Where is the documentation for the goal?