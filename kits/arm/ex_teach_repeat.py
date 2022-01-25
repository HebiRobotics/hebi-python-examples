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
module_names = ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2']
hrdf_file    = "hrdf/A-2303-01.hrdf"
gains_file   = "gains/A-2303-01.xml"


# Create Arm object
arm = arm_api.create([arm_family],
                     names=module_names,
                     hrdf_file=hrdf_file)

mirror_group = lookup.get_group_from_names([arm_family], ['J2B_shoulder1'])
while mirror_group is None:
  print("Looking for double shoulder module...")
  sleep(1)
  mirror_group = lookup.get_group_from_names([arm_family], ['J2B_shoulder1'])

# mirror the position/velocity/effort of module 1 ('J2A_shoulder1') to the module
# in the mirror group ('J2B_shoulder1')
# Keeps the two modules in the double shoulder bracket in sync
arm.add_plugin(arm_api.DoubledJointMirror(1, mirror_group))

arm.load_gains(gains_file)

# mobileIO setup
phone_name = "mobileIO"

# Create mobileIO object
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, arm_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.set_led_color("blue") # as we start in grav comp
m.clear_text() # Clear any garbage on screen
m.update()


# Demo Variables
abort_flag = False
run_mode = "training"
goal = arm_api.Goal(arm.size)

# Print Instructions
instructions = """B1 - Add waypoint (stop)
B2 - Add waypoint (flow)
A3 - Up/down for longer/shorter time to waypoint
B3 - Toggle training/playback
B4 - Clear waypoints
B8 - Quit
"""
print(instructions)
m.add_text(instructions)

#######################
## Main Control Loop ##
#######################

while not abort_flag:
  arm.update() # update the arm
  arm.send()

  if not m.update(0.0):
    print("Failed to get feedback from MobileIO")
    continue

  slider3 = m.get_axis_state(3)

  # B8 - Quit
  if m.get_button_diff(8) == 1: # "ToOn"
    m.set_led_color("transparent")
    m.clear_text()
    abort_flag = True
    break

  if run_mode == "training":
    # B1 - add waypoint (stop)
    if m.get_button_diff(1) == 1: # "ToOn"
      print("Stop waypoint added")
      goal.add_waypoint(t=slider3 + 3.0, position=arm.last_feedback.position, velocity=[0]*arm.size)

    # B2 - add waypoint (flow)
    if m.get_button_diff(2) == 1: # "ToOn"
      print("Flow waypoint added")
      goal.add_waypoint(t=slider3 + 3.0, position=arm.last_feedback.position)

    # B3 - toggle training/playback
    if m.get_button_diff(3) == 1: # "ToOn"
      # Check for more than 2 waypoints
      if goal.waypoint_count > 1:
        print("Switching to playback mode")
        run_mode = "playback"
        m.set_led_color("green")
        arm.set_goal(goal)
      else:
        print("At least two waypoints are needed")

    # B4 - clear waypoints
    if m.get_button_diff(4) == 1: # "ToOn"
      print("Waypoints cleared")
      goal.clear()

  elif run_mode == "playback":
    # B3 toggle training/playback
    if m.get_button_diff(3) == 1: # "ToOn"
      print("Switching to training mode")
      arm.cancel_goal()
      run_mode = "training"
      m.set_led_color("blue")

    # replay through the path again once the goal has been reached
    if arm.at_goal:
      arm.set_goal(goal)
