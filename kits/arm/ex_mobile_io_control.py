#!/usr/bin/env python3

"""
CAUTION: 
This example uses waypoints containing fixed joint angles, which is a bad idea if your actuators have large wind-up. 
The correct way to store waypoints is by using se3 coordinates, and converting them to joint positions using our IK functions.
"""

import hebi
import numpy as np
from time import sleep
from demo_util import create_demo_from_config

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/examples/ex_mobile_io_control.cfg.yaml"

# Set up arm, and mobile_io from config
example_config = hebi.config.load_config(example_config_file)
arm, mobile_io, _ = create_demo_from_config(lookup, example_config)

# Demo Variables
abort_flag = False
goal = hebi.arm.Goal(arm.size)
waypoints = np.asarray(example_config.user_data['waypoints'])

# Print Instructions
instructions = """B1-B3 - Waypoints 1-3
B6 - Grav Comp Mode
B8 - Quit
"""
print(instructions)
mobile_io.clear_text()
mobile_io.add_text(instructions)

#######################
## Main Control Loop ##
#######################

while not abort_flag:
    arm.update()  # update the arm

    if not mobile_io.update():
        print("Failed to get feedback from MobileIO")
        continue

    for N in [1, 2, 3]:
        # BN - Waypoint N (N = 1, 2 , 3)
        if mobile_io.get_button_diff(N) == 1:  # "ToOn"
            mobile_io.set_led_color("green")
            goal.clear()
            goal.add_waypoint(t=example_config.user_data['travel_time'], position=waypoints[N-1])
            arm.set_goal(goal)

    # B6 - Grav Comp
    if mobile_io.get_button_diff(6) == 1:  # "ToOn"
        mobile_io.set_led_color("blue")
        arm.cancel_goal()

    # B8 - Quit
    if mobile_io.get_button_diff(8) == 1:  # "ToOn"
        # Reset text & color, and quit
        mobile_io.clear_text()
        mobile_io.set_led_color("transparent")
        abort_flag = True
        break

    arm.send()
