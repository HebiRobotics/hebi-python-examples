#!/usr/bin/env python3

import os
import hebi
from time import time, sleep
from demo_util import create_demo_from_config

# Set up to find actuators on the network
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/examples/ex_teach_repeat_w_gripper.cfg.yaml"

# Set up arm, mobile_io, and gripper from config
example_config = hebi.config.load_config(example_config_file)
arm, mobile_io, gripper = create_demo_from_config(lookup, example_config)

# Demo Variables
abort_flag = False
pending_goal = False
run_mode = "training"
goal = hebi.arm.Goal(arm.size)
base_travel_time = example_config.user_data['base_travel_time']
min_travel_time = example_config.user_data['min_travel_time']

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
mobile_io.clear_text()
mobile_io.add_text(instructions)

#######################
## Main Control Loop ##
#######################

last_mio_recv = time()

while not abort_flag:
    # If there is a goal pending, set it on the arm and clear the flag
    arm.update()
    arm.send()

    t = time()
    if mobile_io.update(0.0):
        last_mio_recv = t
    else:
        if t - last_mio_recv > 1.0:
            print("Failed to get feedback from MobileIO")
        continue

    slider3 = mobile_io.get_axis_state(3)

    # Check for quit
    if mobile_io.get_button_diff(8) == 1:  # "ToOn"
        mobile_io.set_led_color("transparent")
        mobile_io.clear_text()
        abort_flag = True
        break

    if run_mode == "training":
        # B1 add waypoint (stop)
        if mobile_io.get_button_diff(1) == 1:  # "ToOn"
            print("Stop waypoint added")
            goal.add_waypoint(t= base_travel_time + slider3 * (base_travel_time - min_travel_time), 
                              position=arm.last_feedback.position, velocity=[0] * arm.size)

        # B2 add waypoint (stop) and toggle the gripper
        if mobile_io.get_button_diff(2) == 1:  # "ToOn"
            # Add 2 waypoints to allow the gripper to open or close
            print("Stop waypoint added and gripper toggled")
            position = arm.last_feedback.position
            goal.add_waypoint(t= base_travel_time + slider3 * (base_travel_time - min_travel_time), 
                              position=arm.last_feedback.position, velocity=[0] * arm.size)
            gripper.toggle()

        # B3 add waypoint (flow)
        if mobile_io.get_button_diff(3) == 1:  # "ToOn"
            print("Flow waypoint added")
            goal.add_waypoint(t= base_travel_time + slider3 * (base_travel_time - min_travel_time), 
                              position=arm.last_feedback.position, velocity=[0] * arm.size)

        # B5 toggle training/playback
        if mobile_io.get_button_diff(5) == 1:  # "ToOn"
            # Check for more than 2 waypoints
            if goal.waypoint_count > 1:
                print("Transitioning to playback mode")
                run_mode = "playback"
                mobile_io.set_led_color("green")
                arm.set_goal(goal)
            else:
                print("At least two waypoints are needed")

        # B6 clear waypoints
        if mobile_io.get_button_diff(6) == 1:  # "ToOn"
            print("Waypoints cleared")
            goal.clear()

    elif run_mode == "playback":
        # B5 toggle training/playback
        if mobile_io.get_button_diff(5) == 1:  # "ToOn"
            print("Transitioning to training mode")
            run_mode = "training"
            mobile_io.set_led_color("blue")
            arm.cancel_goal()

        # replay through the path again once the goal has been reached
        if arm.at_goal:
            arm.set_goal(goal)
