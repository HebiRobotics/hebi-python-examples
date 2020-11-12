#!/usr/bin/env python3

from time import sleep

import hebi
import numpy as np

# Example parameters
enable_logging = True
time_per_waypoint = 2 # [s]
grip_time = 2 # [s]

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
  hrdf_file="hrdf/A-2085-06G.hrdf")

# Add Gripper
gripperGroup = lookup.get_group_from_names(families=["Arm"], names=["gripperSpool"])
gripper = hebi.arm.Gripper(gripperGroup, 1, -5)
gripper.load_gains("gains/gripper_spool_gains.xml")
arm.set_end_effector(gripper)

print("")
print("B1 - Add waypoint (stop)")
print("B2 - Add waypoint (stop) and toggle the gripper")
print("B3 - Add waypoint (flow)")
print("B5 - Toggle training/playback")
print("B6 - Clear waypoints")
print("A3 - Up/down for longer/shorter time to waypoint")
print("B8 - Quit")
print("")

# Start background logging
if enable_logging:
  arm.group.start_log(
    directory='logs',
    name='logFile',
    mkdirs=True)

goal = hebi.arm.Goal(arm.size)
zeros = np.zeros(arm.size)
run_mode = "training"
while True:

  arm.update()
  # TODO: there is a bug in Gripper that causes this to always return zero
  #   when there is no active trajectory. Uncomment when fixed.
  # if not arm.update():
  #   print("Failed to update arm")
  #   continue

  if phone.update(timeout_ms=0):
    motion_time = time_per_waypoint + phone.get_axis_state(3)

    # B8 quit example
    if phone.get_button_diff(8) == 3: # "ToOn"
      break

    if run_mode == "training":
      # B1 add waypoint (stop)
      if phone.get_button_diff(1) == 3: # "ToOn"
        print("Stop waypoint added")
        goal.add_waypoint(t=motion_time, position=arm.last_feedback.position, aux=gripper.state, velocity=zeros)

      # B2 add waypoint (stop) and toggle the gripper
      if phone.get_button_diff(2) == 3: # "ToOn"
        # Add 2 waypoints to allow the gripper to open or close
        print("Stop waypoint added and gripper toggled")
        position = arm.last_feedback.position
        goal.add_waypoint(t=motion_time, position=position, aux=gripper.state, velocity=zeros)
        gripper.toggle()
        goal.add_waypoint(t=grip_time, position=position, aux=gripper.state, velocity=zeros)

      # B3 add waypoint (flow)
      if phone.get_button_diff(3) == 3: # "ToOn"
        print("Flow waypoint added")
        goal.add_waypoint(t=motion_time, position=arm.last_feedback.position, aux=gripper.state)

      # B5 toggle training/playback
      if phone.get_button_diff(5) == 3: # "ToOn"
        # Check for more than 2 waypoints
        if goal.waypoint_count > 1:
          print("Transitioning to playback mode")
          run_mode = "playback"
          arm.set_goal(goal)
        else:
          print("At least two waypoints are needed")

      # B6 clear waypoints
      if phone.get_button_diff(6) == 3: # "ToOn"
        print("Waypoints cleared")
        goal.clear()

    elif run_mode == "playback":
      # B5 toggle training/playback
      if phone.get_button_diff(5) == 3: # "ToOn"
        print("Transitioning to training mode")
        run_mode = "training"
        arm.cancel_goal()

  # replay through the path again once the goal has been reached
  if run_mode == "playback" and arm.at_goal:
    arm.set_goal(goal)

  arm.send()

print('Stopped example')
if enable_logging:
  log_file = arm.group.stop_log()
  hebi.util.plot_logs(log_file, 'position', figure_spec=101)
  hebi.util.plot_logs(log_file, 'velocity', figure_spec=102)
  hebi.util.plot_logs(log_file, 'effort', figure_spec=103)