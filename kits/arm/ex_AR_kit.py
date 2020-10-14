#!/usr/bin/env python3

import hebi
import numpy as np
import os
import threading
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api

# Arm setup
phone_family = "HEBI"
phone_name   = "mobileIO"
arm_family   = "Example Arm"
hrdf_file    = "hrdf/A-2085-06.hrdf"

lookup = hebi.Lookup()
sleep(2)

# Setup MobileIO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
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
builder = arm_api.GoalBuilder(arm.size)
control_mode_toggle = 1
quit_demo_button = 8

xyz_target_init = np.asarray([0.5, 0.0, 0.1])
ik_seed_pos = np.asarray([0.01, 1.0, 2.5, 1.5, -1.5, 0.01])
mobile_pos_offset = [0, 0, 0]



def get_mobile_state(quit_demo_button):
  """
  Mobile io function to be run in another thread to prevent main loop stalling on long feedbacks
  """
  global fbk_mobile
  global keep_running
  global run_mode
  global mobile_pos_offset

  m.update()

  m.set_led_color("yellow")
  while not m.get_button_diff(quit_demo_button) == 3: # "ToOn"

    if not m.update():
      print("Failed to get feedback from MobileIO")
      continue
  
    fbk_mobile = m.get_last_feedback()
    # Check for button presses and control state accordingly
    if m.get_button_diff(control_mode_toggle) == 2: # "ToOff"
      run_mode = "startup"
      m.set_led_color("yellow")
    if run_mode == "standby":
      m.set_led_color("green")
    if m.get_button_diff(control_mode_toggle) == 3 and run_mode == "standby": # "ToOn"
      m.set_led_color("blue")
      run_mode = "control"
      mobile_pos_offset = xyz_target_init - fbk_mobile.ar_position[0]
            
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
    arm.set_goal([joint_targets])
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
    phone_target_xyz = fbk_mobile.ar_position[0] + mobile_pos_offset
    joint_targets = arm.ik_target_xyz(arm.last_feedback.position, phone_target_xyz)
    arm.set_goal([joint_targets], times=[1.0])
    
  arm.send()
