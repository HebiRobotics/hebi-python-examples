#!/usr/bin/env python3

import hebi
import numpy as np
import os
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api

# Arm setup
phone_family = "HEBI"
phone_name   = "mobileIO"
arm_family   = "Arm"
hrdf_file    = "hrdf/A-2085-06.hrdf"

gripper_family = arm_family
gripper_name   = 'gripperSpool'

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
gripper = arm_api.Gripper(lookup.get_group_from_names([gripper_family], [gripper_name]), 1, -5)
gripper.load_gains("gains/gripper_spool_gains.xml")

arm.set_end_effector(gripper)

keep_running = True
pending_goal = False
run_mode = "training"
goal = arm_api.Goal(arm.size)

print("")
print("B1 - Add waypoint (stop)")
print("B2 - Add waypoint (stop) and toggle the gripper")
print("B3 - Add waypoint (flow)")
print("B5 - Toggle training/playback")
print("B6 - Clear waypoints")
print("A3 - Up/down for longer/shorter time to waypoint")
print("B8 - Quit")
print("")

while keep_running:
  # If there is a goal pending, set it on the arm and clear the flag
  if pending_goal:
    arm.set_goal(goal)
    pending_goal = False

  if not arm.update():
    print("Failed to update arm")
    continue

  if not m.update():
    print("Failed to get feedback from MobileIO")

    slider3 = m.get_axis_state(3)

    # Check for quit
    if m.get_button_diff(8) == 3: # "ToOn"
      keep_running = False
      break

    if run_mode == "training":
      # B1 add waypoint (stop)
      if m.get_button_diff(1) == 3: # "ToOn"
        print("Stop waypoint added")
        goal.add_waypoint(t=slider3 + 4.0, position=arm.last_feedback.position, aux=gripper.state, velocity=0)

      # B2 add waypoint (stop) and toggle the gripper
      if m.get_button_diff(2) == 3: # "ToOn"
        # Add 2 waypoints to allow the gripper to open or close
        print("Stop waypoint added and gripper toggled")
        position = arm.last_feedback.position
        goal.add_waypoint(t=slider3 + 4.0, position=position, aux=gripper.state, velocity=0)
        gripper.toggle()
        goal.add_waypoint(t=2.0, position=position, aux=gripper.state, velocity=0)

      # B3 add waypoint (flow)
      if m.get_button_diff(3) == 3: # "ToOn"
        print("Flow waypoint added")
        goal.add_waypoint(t=slider3 + 4.0, position=arm.last_feedback.position, aux=gripper.state)

      # B5 toggle training/playback
      if m.get_button_diff(5) == 3: # "ToOn"
        # Check for more than 2 waypoints
        if goal.waypoint_count > 1:
          print("Transitioning to playback mode")
          run_mode = "playback"
          pending_goal = True
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
        arm.cancel_goal()

  # replay through the path again once the goal has been reached
  if run_mode == "playback" and arm.at_goal:
    arm.set_goal(goal)

  arm.send()
