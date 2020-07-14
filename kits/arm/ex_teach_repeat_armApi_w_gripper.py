#!/usr/bin/env python3

import arm
import hebi
from hebi.util import create_mobile_io
from time import sleep


# Set up arm
family_name  = "Arm"
module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
hrdf = "hrdf/A-2085-06G.hrdf"
gripper_name = "gripperSpool"
p = arm.ArmParams(family_name, module_names, hrdf, gripperName=gripper_name)
a = arm.Arm(p)
a.loadGains("gains/A-2085-06.xml")
a.gripper.loadGains("gains/gripper_spool_gains.xml")
a.gripper.open()


# Mobile device setup
phone_family = 'Arm'
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
grip_states = []
flow = []
durations = []
run_mode = "training"

print("")
print("B1 - Add waypoint (stop)")
print("B2 - Add waypoint (stop) and toggle the gripper")
print("B3 - Add waypoint (flow)")
print("B5 - Toggle training/playback")
print("B6 - Clear waypoints")
print("A3 - Up/down for longer/shorter time to waypoint")
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

  # B8 Check for quit
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
        grip_states.append(a.gripper.state)

      # B2 add waypoint (stop) and toggle the gripper
      if m.get_button_diff(2) == 3:
        # Add 2 waypoints to allow the gripperr to open or close
        print("Stop waypoint added and gripper toggled")

        # first waypoint
        waypoints.append(a.fbk.position)
        flow.append(False)
        grip_states.append(a.gripper.state)
        durations.append(slider3 + 4)

        # toggle the gripper
        a.gripper.toggle()

        # second waypoint
        waypoints.append(a.fbk.position)
        flow.append(False)
        grip_states.append(a.gripper.state) # this will now be the toggled state
        durations.append(2) # time given to gripper for closing


      # B3 add waypoint (flow)
      if m.get_button_diff(3) == 3: # "ToOn"
        print("Flow waypoint added")
        waypoints.append(a.fbk.position)
        flow.append(True)
        durations.append(slider3 + 4)
        grip_states.append(a.gripper.state)

      # B5 toggle training/playback
      if m.get_button_diff(5) == 3: # "ToOn"
        # Check for more than 2 waypoints
        if len(waypoints) > 1:
          print("Starting playback of waypoints")
          run_mode = "playback"
          playback_waypoint = 0 # reset playback_waypoint before starting playback
          a.gripper.open()
          a.send()
          continue
        else:
          print("At least two waypoints are needed")

      # B6 clear waypoints
      if m.get_button_diff(6) == 3: # "ToOn"
        print("Waypoints cleared")
        waypoints = []
        flow = []
        durations = []
        grip_states = []

      

  if run_mode == "playback":

    # B5 toggle training/playback, leave playback
    if m.get_button_diff(5) == 3: # "ToOn"
      print("Returned to training mode")
      run_mode = "training"
      a.cancelGoal()
    
    if a.at_goal:
      print("Reached waypoint number:", playback_waypoint) # waypoint 0 is position from where you start the playback
      
      if playback_waypoint == len(waypoints): # finished playback, reset counter and restart
        playback_waypoint = 0
      
      # Set up next waypoint according to waypoint counter
      next_waypoint = waypoints[playback_waypoint]
      next_flow = flow[playback_waypoint]
      next_duration = durations[playback_waypoint]

      # Send the new commands
      a.createGoal([next_waypoint], flow=[next_flow], duration=[next_duration])
      a.setGoal()
      a.gripper.setState(grip_states[playback_waypoint])

      # Iterate waypoint counter 
      playback_waypoint += 1
    
  a.send()
