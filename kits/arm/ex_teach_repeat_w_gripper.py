#!/usr/bin/env python3

import hebi
import numpy as np
from time import sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io

# Set up to find actuators on the network
lookup = hebi.Lookup()
sleep(2)

# Arm setup
arm_family   = "Arm"
module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
hrdf_file    = "hrdf/A-2085-06G.hrdf"
gains_file   = "gains/A-2085-06.xml"

# Create Arm object
arm = arm_api.create([arm_family],
                     names=module_names,
                     hrdf_file=hrdf_file)
arm.load_gains(gains_file)

# Add the gripper 
gripper_family = arm_family
gripper_name   = 'gripperSpool'
gripper = arm_api.Gripper(lookup.get_group_from_names([gripper_family], [gripper_name]), -5, 1)
gripper.load_gains("gains/gripper_spool_gains.xml")
arm.set_end_effector(gripper)

# Create mobileIO object
phone_name = "mobileIO"
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, arm_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.set_led_color("blue") # as we start in grav comp
m.clear_text() # Clear any garbage on screen
m.update()

# Demo Variables
abort_flag = False
pending_goal = False
run_mode = "training"
goal = arm_api.Goal(arm.size)

# Print Instructions
instructions = """B1 - Add waypoint (stop)
B2 - Add waypoint (stop) and toggle the gripper
B3 - Add waypoint (flow)
B5 - Toggle training/playback
B6 - Clear waypoints
A3 - Up/down for longer/shorter time to waypoint
B8 - Quit
"""
print(instructions)
m.set_text(instructions)

#######################
## Main Control Loop ##
#######################

while not abort_flag:
  # If there is a goal pending, set it on the arm and clear the flag
  arm.update()
    
  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  slider3 = m.get_axis_state(3)

  # Check for quit
  if m.get_button_diff(8) == 3: # "ToOn"
    m.set_led_color("transparent")
    m.clear_text()
    abort_flag = True
    break

  if run_mode == "training":
    # B1 add waypoint (stop)
    if m.get_button_diff(1) == 3: # "ToOn"
      print("Stop waypoint added")
      goal.add_waypoint(t=slider3 + 3.0, position=arm.last_feedback.position, aux=gripper.state, velocity=[0]*arm.size)

    # B2 add waypoint (stop) and toggle the gripper
    if m.get_button_diff(2) == 3: # "ToOn"
      # Add 2 waypoints to allow the gripper to open or close
      print("Stop waypoint added and gripper toggled")
      position = arm.last_feedback.position
      goal.add_waypoint(t=slider3 + 3.0, position=position, aux=gripper.state, velocity=[0]*arm.size)
      gripper.toggle()
      goal.add_waypoint(t=2.0, position=position, aux=gripper.state, velocity=[0]*arm.size)

    # B3 add waypoint (flow)
    if m.get_button_diff(3) == 3: # "ToOn"
      print("Flow waypoint added")
      goal.add_waypoint(t=slider3 + 3.0, position=arm.last_feedback.position, aux=gripper.state)

    # B5 toggle training/playback
    if m.get_button_diff(5) == 3: # "ToOn"
      # Check for more than 2 waypoints
      if goal.waypoint_count > 1:
        print("Transitioning to playback mode")
        run_mode = "playback"
        m.set_led_color("green")
        arm.set_goal(goal)
      else:
        print("At least two waypoints are needed")

    # B6 clear waypoints
    if m.get_button_diff(6) == 3: # "ToOn"
      print("Waypoints cleared")
      goal.clear()

  elif run_mode == "playback":    
    # B5 toggle training/playback
    if m.get_button_diff(5) == 3: # "ToOn"
      print("Transitioning to training mode")
      run_mode = "training"
      m.set_led_color("blue")
      arm.cancel_goal()

    # replay through the path again once the goal has been reached
    if arm.at_goal:
      arm.set_goal(goal)

  arm.send()
