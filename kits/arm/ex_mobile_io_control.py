#!/usr/bin/env python3

import hebi
import numpy as np
import os
import sys
from time import sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io

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
m.set_button_mode(1, 'momentary')
m.set_button_mode(2, 'momentary')
m.set_button_mode(3, 'momentary')
m.update()

# Setup arm components
arm = arm_api.create([arm_family],
                     names=['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
                     lookup=lookup,
                     hrdf_file=hrdf_file)

quit_demo_button = 8
keep_running = True
run_mode = "points"
goal = arm_api.Goal(arm.size)
first_run = True
point_1 = np.asarray([np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, np.pi/4, 0], dtype=np.float64)
point_2 = np.asarray([-np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, 3*np.pi/4, 0], dtype=np.float64)
point_3 = np.asarray([0, 0, 0, 0, 0, 0], dtype=np.float64)

print('B1-3 - select different points to move the arm to.')
print('B6 - enables grav comp mode.')
print('B8 - Quits the demo.')
                           
while keep_running:

  if not arm.update():
    print("Failed to update arm")
    continue

  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  # Check for quit
  if m.get_button_state(quit_demo_button):
    # Set led red and quit
    m.set_led_color("red")
    keep_running = False
    break

  # On first run go to point 1
  if first_run:
    arm.set_goal(goal.clear().add_waypoint(t=4, position=point_1))
    first_run = False

  # B1 point 1
  elif m.get_button_diff(1) == 3: # "ToOn"
    arm.set_goal(goal.clear().add_waypoint(t=4, position=point_1))

  # B2 point 2
  elif m.get_button_diff(2) == 3: # "ToOn"
    arm.set_goal(goal.clear().add_waypoint(t=4, position=point_2))

  # B3 point 3
  elif m.get_button_diff(3) == 3: # "ToOn"
    arm.set_goal(goal.clear().add_waypoint(t=4, position=point_3))

  # B6 grav comp
  elif m.get_button_diff(6) == 3: # "ToOn"
    run_mode = "grav comp"
    arm.cancel_goal()

  arm.send()

  # Set led to blue for grav comp mode, green for points mode
  m.set_led_color("green" if (run_mode == "points") else "blue")
