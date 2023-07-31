#!/usr/bin/env python3

import hebi
import numpy as np
from time import sleep
from hebi import arm as arm_api
from hebi.util import create_mobile_io

# Arm setup
arm_family = "Arm"
module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
hrdf_file = "hrdf/A-2085-06.hrdf"
gains_file = "gains/A-2085-06.xml"


# Create Arm object
arm = arm_api.create([arm_family],
                     names=module_names,
                     hrdf_file=hrdf_file)
arm.load_gains(gains_file)


# mobileIO setup
phone_name = "mobileIO"
lookup = hebi.Lookup()
print('Waiting for Mobile IO device to come online...')
sleep(2)

# Create mobileIO object
m = create_mobile_io(lookup, arm_family, phone_name)
if m is None:
    raise RuntimeError("Could not find Mobile IO device")

m.send_layout('./layouts/ex_mobile_io_control.json')
m.set_led_color("blue")  # as we start in grav comp
m.update()


# Demo Variables
abort_flag = False
goal = arm_api.Goal(arm.size)
waypoint_1 = np.asarray([0, 0, 0, 0, 0, 0], dtype=np.float64)
waypoint_2 = np.asarray([np.pi / 4, np.pi / 3, 2 * np.pi / 3, np.pi / 3, np.pi / 4, 0], dtype=np.float64)
waypoint_3 = np.asarray([-np.pi / 4, np.pi / 3, 2 * np.pi / 3, np.pi / 3, 3 * np.pi / 4, 0], dtype=np.float64)

# Print Instructions
instructions = """
B1 - Add Waypoint 1
B2 - Add Waypoint 2
B3 - Add Waypoint 3
B4 - Grav Comp Mode
B5 - Quit
"""
print(instructions)

#######################
## Main Control Loop ##
#######################

while not abort_flag:
    arm.update()  # update the arm

    if not m.update():
        print("Failed to get feedback from MobileIO")
        continue

    # B1 - Waypoint 1
    if m.get_button_diff(1) == 1:  # "ToOn"
        m.set_led_color("green")
        goal.clear()
        goal.add_waypoint(t=3, position=waypoint_1)
        arm.set_goal(goal)

    # B2 - Waypoint 2
    if m.get_button_diff(2) == 1:  # "ToOn"
        m.set_led_color("green")
        goal.clear()
        goal.add_waypoint(t=3, position=waypoint_2)
        arm.set_goal(goal)

    # B3 - Waypoint 3
    if m.get_button_diff(3) == 1:  # "ToOn"
        m.set_led_color("green")
        goal.clear()
        goal.add_waypoint(t=3, position=waypoint_3)
        arm.set_goal(goal)

    # B6 - Grav Comp
    if m.get_button_diff(4) == 1:  # "ToOn"
        m.set_led_color("blue")
        arm.cancel_goal()

    # B8 - Quit
    if m.get_button_diff(5) == 1:  # "ToOn"
        # Reset text & color, and quit
        m.clear_text()
        m.set_led_color("transparent")
        abort_flag = True
        break

    arm.send()
