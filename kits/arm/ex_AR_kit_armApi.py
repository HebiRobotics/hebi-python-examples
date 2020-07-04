#!/usr/bin/env python3

import numpy as np
import threading
from time import sleep


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
import os, sys
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


import hebi
from hebi.robot_model import endeffector_position_objective
from hebi.util import create_mobile_io
import arm

# Set up arm
family_name = "Example Arm"
module_names = ("J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3")
hrdf = "hrdf/6-DoF_arm.hrdf"
p = arm.ArmParams(family_name, module_names, hrdf)
a = arm.Arm(p)

# Set up our mobile io interface
phone_family = "HEBI"
phone_name = "mobileIO"

lookup = hebi.Lookup()
sleep(2)

print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.set_button_mode(1, 'toggle')
m.update()

fbk_mobile = m.get_last_feedback()

control_mode_toggle = 1
quit_demo_button = 8

abort_flag = False
run_mode = "startup"

def get_mobile_state(quit_demo_button):
  """
  Mobile io function to be run in another thread to prevent main loop stalling on long feedbacks
  """
  global fbk_mobile
  global abort_flag
  global run_mode
  global mobile_pos_offset
   
  m.set_led_color("yellow")
  while not m.get_button_diff(quit_demo_button) == 3: # "ToOn"
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
  abort_flag = True
  return


# Start mobile io thread
t1 = threading.Thread(target=get_mobile_state, args=(quit_demo_button,), daemon=True)
t1.start()


def get_ik(xyz_target, ik_seed):
  """
  Helper function to rebuild the IK solver with the appropriate objective functions
  """
  return a.model.solve_inverse_kinematics(ik_seed, endeffector_position_objective(xyz_target))


# Startup coordinates
xyz_target_init = np.asarray([0.5, 0.0, 0.1])
ik_seed_pos = np.asarray([0.01, 1.0, 2.5, 1.5, -1.5, 0.01])
mobile_pos_offset = [0, 0, 0]


# Main run loop
while not abort_flag:
  a.update()
  # Update MobileIO state
  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  if run_mode == "startup":
    # Move to starting pos
    joint_targets = get_ik(xyz_target_init, ik_seed_pos)
    a.createGoal([joint_targets])
    a.setGoal()
    run_mode = "moving to start pos"
    
  if run_mode == "moving to start pos":
    # When at startup pos, switch to standby mode
    if a.at_goal:
      run_mode = "standby"
    
  if run_mode == "standby":
    # Wait for mobile io input while holding arm in place
    a.send()
    continue
    
  if run_mode == "control":
    # Follow phone's motion in 3D space
    phone_target_xyz = fbk_mobile.ar_position[0] + mobile_pos_offset
    joint_targets = get_ik(phone_target_xyz, a.fbk.position)
    a.createGoal([joint_targets], durration=[1])
    a.setGoal()
    
  a.send()
