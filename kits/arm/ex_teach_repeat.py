#!/usr/bin/env python3

import hebi
from time import time, sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/examples/ex_teach_repeat.cfg"

# Set up arm from config
example_config = hebi.config.load_config(example_config_file)
arm = hebi.arm.create_from_config(example_config)

# Set up Mobile IO from config
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, example_config.mobile_io['family'], example_config.mobile_io['name'])
if m is None:
    raise RuntimeError("Could not find Mobile IO device")
m.send_layout(example_config.mobile_io['layout'])
m.set_button_mode(2, 'toggle')
m.update()

# Demo Variables
abort_flag = False
run_mode = "training"
goal = arm_api.Goal(arm.size)
base_travel_time = example_config.user_data['base_travel_time']
min_travel_time = example_config.user_data['min_travel_time']

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
            goal.add_waypoint(t= base_travel_time + slider3 * (base_travel_time - min_travel_time), 
                              position=arm.last_feedback.position, velocity=[0] * arm.size)
            
        print(base_travel_time + slider3 * (base_travel_time - min_travel_time))

        # B2 - add waypoint (flow)
        if m.get_button_diff(2) == 1:  # "ToOn"
            print("Flow waypoint added")
            goal.add_waypoint(t= base_travel_time + slider3 * (base_travel_time - min_travel_time), 
                              position=arm.last_feedback.position)

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
