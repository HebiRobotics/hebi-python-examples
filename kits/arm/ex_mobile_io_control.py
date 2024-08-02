#!/usr/bin/env python3

"""
CAUTION: 
This example uses waypoints containing fixed joint angles, which is a bad idea if your actuators have large wind-up. 
The correct way to store waypoints is by using se3 coordinates, and converting them to joint positions using our IK functions.
"""

import hebi
import numpy as np
from time import sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io_from_config

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
m = create_mobile_io_from_config(lookup, example_config)
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
