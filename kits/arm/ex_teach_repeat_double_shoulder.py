#!/usr/bin/env python3

import hebi
from time import time, sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io

# Set up to find actuators on the network
lookup = hebi.Lookup()
sleep(2)

# Arm setup
arm_family = "Arm"
module_names = ['J1_base', 'J2A_shoulder1', 'J2B_shoulder2', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
hrdf_file = "hrdf/A-2099-07.hrdf"
gains_file = "gains/A-2099-07.xml"


# Create Arm object
arm = arm_api.create([arm_family],
                     names=module_names,
                     hrdf_file=hrdf_file,
                     lookup=lookup)

alt_shoulder_group = lookup.get_group_from_names(arm_family, ['J2B_shoulder1'])
double_shoulder = arm_api.DoubleJointedMirror(2, alt_shoulder_group)
arm.add_plugin(double_shoulder)

arm.load_gains(gains_file)

# mobileIO setup
phone_name = "mobileIO"

# Create mobileIO object
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, arm_family, phone_name)
if m is None:
    raise RuntimeError("Could not find Mobile IO device")
m.set_led_color("blue")  # as we start in grav comp
m.clear_text()  # Clear any garbage on screen
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
m.clear_text()
m.add_text(instructions)

#######################
## Main Control Loop ##
#######################

last_mio_recv = time()

while not abort_flag:
    arm.update()  # update the arm
    arm.send()

    t = time()
    if m.update(0.0):
        last_mio_recv = t
    else:
        if t - last_mio_recv > 1.0:
            print("Failed to get feedback from MobileIO")
        continue

    slider3 = m.get_axis_state(3)

    # B8 - Quit
    if m.get_button_diff(8) == 1:  # "ToOn"
        m.set_led_color("transparent")
        m.clear_text()
        abort_flag = True
        break

    if run_mode == "training":
        # B1 - add waypoint (stop)
        if m.get_button_diff(1) == 1:  # "ToOn"
            print("Stop waypoint added")
            goal.add_waypoint(t=slider3 + 3.0, position=arm.last_feedback.position, velocity=[0] * arm.size)

        # B2 - add waypoint (flow)
        if m.get_button_diff(2) == 1:  # "ToOn"
            print("Flow waypoint added")
            goal.add_waypoint(t=slider3 + 3.0, position=arm.last_feedback.position)

        # B3 - toggle training/playback
        if m.get_button_diff(3) == 1:  # "ToOn"
            # Check for more than 2 waypoints
            if goal.waypoint_count > 1:
                print("Switching to playback mode")
                run_mode = "playback"
                m.set_led_color("green")
                arm.set_goal(goal)
            else:
                print("At least two waypoints are needed")

        # B4 - clear waypoints
        if m.get_button_diff(4) == 1:  # "ToOn"
            print("Waypoints cleared")
            goal.clear()

    elif run_mode == "playback":
        # B3 toggle training/playback
        if m.get_button_diff(3) == 1:  # "ToOn"
            print("Switching to training mode")
            arm.cancel_goal()
            run_mode = "training"
            m.set_led_color("blue")

        # replay through the path again once the goal has been reached
        if arm.at_goal:
            arm.set_goal(goal)
