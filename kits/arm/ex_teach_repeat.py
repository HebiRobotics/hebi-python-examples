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

# Lookup initialization
lookup = hebi.Lookup()
sleep(2)

# Setup Mobile IO input device
phone = hebi.util.create_mobile_io(
  lookup=lookup,
  family="HEBI",
  name="mobileIO")

if phone is None:
  raise RuntimeError("Could not find Mobile IO device")
phone.update()

print("")
print("B1 - Add waypoint (stop)")
print("B2 - Add waypoint (flow)")
print("A3 - Up/down for longer/shorter time to waypoint")
print("B3 - Toggle training/playback")
print("B4 - Clear waypoints")
print("B8 - Quit")
print("")

# Setup Arm
arm = hebi.arm.create(
  lookup=lookup,
  families=["Arm"],
  names=['J1_base',
         'J2_shoulder',
         'J3_elbow',
         'J4_wrist1',
         'J5_wrist2',
         'J6_wrist3'],
  hrdf_file="hrdf/A-2085-06.hrdf")

time_per_waypoint = 2 # [s]
zeros = np.zeros(arm.size)
goal = arm_api.Goal(arm.size)
run_mode = "training"
while True:

  if not arm.update():
    print("Failed to update arm")
    continue

  if phone.update(timeout_ms=0):
    time_offset = phone.get_axis_state(3)

    # Check for quit
    if phone.get_button_diff(8) == 3: # "ToOn"
      break

    if run_mode == "training":
      # B1 add waypoint (stop)
      if phone.get_button_diff(1) == 3: # "ToOn"
        print("Stop waypoint added")
        goal.add_waypoint(t=time_per_waypoint + time_offset, position=arm.last_feedback.position, velocity=zeros, acceleration=zeros)

      # B2 add waypoint (flow)
      if phone.get_button_diff(2) == 3: # "ToOn"
        print("Flow waypoint added")
        goal.add_waypoint(t=time_per_waypoint + time_offset, position=arm.last_feedback.position)

      # B3 toggle training/playback
      if phone.get_button_diff(3) == 3: # "ToOn"
        # Check for more than 2 waypoints
        if goal.waypoint_count > 1:
          print("Transitioning to playback mode")
          run_mode = "playback"
          arm.set_goal(goal)
        else:
          print("At least two waypoints are needed")

      # B4 clear waypoints
      if phone.get_button_diff(4) == 3: # "ToOn"
        print("Waypoints cleared")
        goal.clear()

    elif run_mode == "playback":
      # B3 toggle training/playback
      if phone.get_button_diff(3) == 3: # "ToOn"
        print("Transitioning to training mode")
        run_mode = "training"
        arm.cancel_goal()

      # replay through the path again once the goal has been reached
      if arm.at_goal:
        arm.set_goal(goal)

  arm.send()
