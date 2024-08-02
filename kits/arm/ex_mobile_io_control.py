#!/usr/bin/env python3

import hebi
import numpy as np
from time import sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/examples/ex_mobile_io_control.cfg"

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
goal = arm_api.Goal(arm.size)
waypoints = np.asarray(example_config.user_data['waypoints'])

# Print Instructions
instructions = """B1-B3 - Waypoints 1-3
B6 - Grav Comp Mode
B8 - Quit
"""
print(instructions)
m.clear_text()
m.add_text(instructions)

#######################
## Main Control Loop ##
#######################

while not abort_flag:
    arm.update()  # update the arm

    if not m.update():
        print("Failed to get feedback from MobileIO")
        continue

    for N in [1, 2, 3]:
        # BN - Waypoint N (N = 1, 2 , 3)
        if m.get_button_diff(N) == 1:  # "ToOn"
            m.set_led_color("green")
            goal.clear()
            goal.add_waypoint(t=example_config.user_data['travel_time'], position=waypoints[N-1])
            arm.set_goal(goal)

    # B6 - Grav Comp
    if m.get_button_diff(6) == 1:  # "ToOn"
        m.set_led_color("blue")
        arm.cancel_goal()

    # B8 - Quit
    if m.get_button_diff(8) == 1:  # "ToOn"
        # Reset text & color, and quit
        m.clear_text()
        m.set_led_color("transparent")
        abort_flag = True
        break

    arm.send()
