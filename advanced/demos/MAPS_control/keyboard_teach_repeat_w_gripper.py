#!/usr/bin/env python3

import hebi
from time import time, sleep
from hebi import arm as arm_api
from kbhit import KBHit

# Set up to find actuators on the network
lookup = hebi.Lookup()
sleep(2)

# Arm setup
arm_family = "Arm"
module_names = ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2']
hrdf_file = "hrdf/A-2303-01.hrdf"
gains_file = "gains/A-2303-01.xml"

# Create Arm object
arm = arm_api.create([arm_family],
                     names=module_names,
                     hrdf_file=hrdf_file,
                     lookup=lookup)

mirror_group = lookup.get_group_from_names([arm_family], ['J2B_shoulder1'])
while mirror_group is None:
    print('Still looking for mirror group...')
    sleep(1)
    mirror_group = lookup.get_group_from_names([arm_family], ['J2B_shoulder1'])
# mirror the position/velocity/effort of module 1 ('J2A_shoulder1') to the module
# in the mirror group ('J2B_shoulder1')
# Keeps the two modules in the double shoulder bracket in sync
arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

arm.load_gains(gains_file)

# Add the gripper
gripper_family = arm_family
gripper_name = 'gripperSpool'

gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])
while gripper_group is None:
    print(f"Looking for gripper module {gripper_family} / {gripper_name} ...")
    sleep(1)
    gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])

gripper = arm_api.Gripper(gripper_group, -5, 1)
gripper.load_gains("gains/gripper_spool_gains.xml")
arm.set_end_effector(gripper)

# Demo Variables
abort_flag = False
pending_goal = False
run_mode = "training"
goal = arm_api.Goal(arm.size)

# Print Instructions
instructions = """'s' - Add waypoint (stop)
'g' - Add waypoint (stop) and toggle the gripper
'f' - Add waypoint (flow)
'Spacebar' - Toggle training/playback
'c' - Clear waypoints
'</>' - Up/down for longer/shorter time to waypoint
'Esc' - Quit
"""
print(instructions)

#######################
## Main Control Loop ##
#######################

kb = KBHit()

waypoint_speed = 0.0

while not abort_flag:
    # If there is a goal pending, set it on the arm and clear the flag
    arm.update()
    arm.send()

    t = time()
    char = None
    if kb.kbhit():
        char = kb.getch()

    if char == '.':
        waypoint_speed += 0.1
        if waypoint_speed > 1.0:
            waypoint_speed = 1.0
    elif char == ',':
        waypoint_speed -= 0.1
        if waypoint_speed < -1.0:
            waypoint_speed = -1.0

    # Check for quit
    if char is not None and ord(char) == 27:  # Esc
        abort_flag = True
        break

    if run_mode == "training":
        # Add waypoint (stop)
        if char == 's':
            print("Stop waypoint added")
            goal.add_waypoint(t=waypoint_speed + 3.0, position=arm.last_feedback.position, aux=gripper.state, velocity=[0] * arm.size)

        # Add waypoint (stop) and toggle the gripper
        if char == 'g':
            # Add 2 waypoints to allow the gripper to open or close
            print("Stop waypoint added and gripper toggled")
            position = arm.last_feedback.position
            goal.add_waypoint(t=waypoint_speed + 3.0, position=position, aux=gripper.state, velocity=[0] * arm.size)
            gripper.toggle()
            goal.add_waypoint(t=2.0, position=position, aux=gripper.state, velocity=[0] * arm.size)

        # Add waypoint (flow)
        if char == 'f':
            print("Flow waypoint added")
            goal.add_waypoint(t=waypoint_speed + 3.0, position=arm.last_feedback.position, aux=gripper.state)

        # Toggle training/playback
        if char is not None and ord(char) == 32:  # Spacebar
            # Check for more than 2 waypoints
            if goal.waypoint_count > 1:
                print("Transitioning to playback mode")
                run_mode = "playback"
                arm.set_goal(goal)
            else:
                print("At least two waypoints are needed")

        # Clear waypoints
        if char == 'c':
            print("Waypoints cleared")
            goal.clear()

    elif run_mode == "playback":
        # Toggle training/playback
        if char is not None and ord(char) == 32:  # Spacebar
            print("Transitioning to training mode")
            run_mode = "training"
            arm.cancel_goal()

        # replay through the path again once the goal has been reached
        if arm.at_goal:
            arm.set_goal(goal)
