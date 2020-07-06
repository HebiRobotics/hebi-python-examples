#!/usr/bin/env python3

import arm
import hebi
from hebi.util import create_mobile_io
from time import sleep


# Set up arm
family_name  = "Example Arm"
module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
hrdf = "hrdf/6-DoF_arm.hrdf"
p = arm.ArmParams(family_name, module_names, hrdf)
a = arm.Arm(p)

# Mobile device setup
phone_family = 'HEBI'
phone_name   = "mobileIO"

lookup = hebi.Lookup()
sleep(2)

print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.update()

abort_flag = False

waypoints = []
flow = []
durations = []
run_mode = "training"
curr_waypoint_num = 0

print("")
print("B1 - Add waypoint (stop)")
print("B2 - Add waypoint (flow)")
print("A3 - Up/down for longer/shorter time to waypoint")
print("B3 - Toggle training/playback")
print("B4 - Clear waypoints")
print("B8 - Quit")
print("")

while not abort_flag:
  # Update arm and mobile io
  a.update()
  # Update button states
  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  slider3 = m.get_axis_state(3)

  # Check for quit
  if m.get_button_diff(8) == 3: # "ToOn"
      abort_flag = True
      break

  if run_mode == "training":
      # B1 add waypoint (stop)
      if m.get_button_diff(1) == 3: # "ToOn"
        print("Stop waypoint added")
        waypoints.append(a.fbk.position)
        flow.append(False)
        durations.append(slider3 + 4)

      # B2 add waypoint (flow)
      if m.get_button_diff(2) == 3: # "ToOn"
        print("Flow waypoint added")
        waypoints.append(a.fbk.position)
        flow.append(True)
        durations.append(slider3 + 4)

      # B3 toggle training/playback
      if m.get_button_diff(3) == 3: # "ToOn"
        # Check for more than 2 waypoints
        if len(waypoints) > 1:
          run_mode = "playback"
          curr_waypoint_num = 0
          a.send()
          continue
        else:
          print("At least two waypoints are needed")

      # B4 clear waypoints
      if m.get_button_diff(4) == 3: # "ToOn"
        print("Waypoints cleared")
        waypoints = []
        flow = []
        durations = []

  if run_mode == "playback":
    # B3 toggle training/playback
    if m.get_button_diff(3) == 3: # "ToOn"
      run_mode = "training"
      a.cancelGoal()

    # When not running through waypoints, run through the waypoints
    if a.at_goal:
      a.createGoal(waypoints, flow=flow, duration=durations)
      a.setGoal()

  a.send()
