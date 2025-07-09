#!/usr/bin/env python3

import hebi
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
from hebi_util import create_mobile_io_from_config

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/ex_AR_kit.cfg.yaml"   # Relative to this file directory
example_config = hebi.config.load_config(os.path.join(os.path.dirname(os.path.realpath(__file__)), example_config_file))

# Set up arm, and mobile_io from config
arm = hebi.arm.create_from_config(example_config, lookup)
while not (mobile_io := create_mobile_io_from_config(example_config, lookup)):
    print('Looking for mobileIO device')
    sleep(1)

# Demo Variables
abort_flag = False
run_mode = "softstart"
goal = hebi.arm.Goal(arm.size)

# Command the softstart to the home position
softstart = hebi.arm.Goal(arm.size)
softstart.add_waypoint(t=example_config.user_data['homing_duration'], 
                       position=example_config.user_data['home_position'])
arm.update()
arm.set_goal(softstart)
arm.send()

# Get the cartesian position and rotation matrix @ home position
xyz_home = np.zeros(3)
rot_home = np.zeros((3, 3))
arm.FK(example_config.user_data['home_position'], xyz_out=xyz_home, orientation_out=rot_home)

# Get the states for the mobile device
xyz_phone_init = np.zeros(3)
rot_phone_init = np.zeros((3, 3))

# Print instructions
instructions = """AR KIT EXAMPLE

    🏠 - Home
    📲 - AR Control
    🌍 - Grav Comp
    ❌ - Quit"""

print(instructions)
mobile_io.add_text(instructions)

#######################
## Main Control Loop ##
#######################

home_btn = 1
ar_btn = 3
gravcomp_btn = 6
quit_btn = 8


xyz_scale = np.array(example_config.user_data['xyz_scale'])

while not abort_flag:
    arm.update()  # update the arm

    if run_mode == "softstart":
        # End softstart when the arm reaches the home_position
        if arm.at_goal:
            mobile_io.set_led_color("yellow")
            run_mode = "waiting"
            continue
        arm.send()
        continue

    if not mobile_io.update(timeout_ms=0):
        # print("Failed to get feedback from MobileIO")
        arm.send()
        continue

    # Quit
    elif mobile_io.get_button_diff(quit_btn) == 1:
        mobile_io.set_led_color("transparent")
        abort_flag = True
        break

    # Return to home position
    elif mobile_io.get_button_diff(home_btn) == 1:
        mobile_io.set_led_color("yellow")
        run_mode = "waiting"
        arm.set_goal(softstart)

    # Start AR Control
    elif mobile_io.get_button_diff(ar_btn) == 1 and run_mode != "ar_mode":
        mobile_io.set_led_color("green")
        run_mode = "ar_mode"

        # Store initial position and orientation as baseline
        xyz_phone_init = mobile_io.position.copy()
        wxyz = mobile_io.orientation
        xyzw = [*wxyz[1:], wxyz[0]]
        rot_phone_init = R.from_quat(xyzw).as_matrix()

    # Grav Comp Mode
    elif mobile_io.get_button_diff(gravcomp_btn) == 1:  # "ToOn"
        mobile_io.set_led_color("blue")
        run_mode = "grav_comp"
        arm.cancel_goal()

    if run_mode == "ar_mode":
        # Get the latest mobile position and orientation
        xyz_phone = mobile_io.position
        wxyz = mobile_io.orientation
        xyzw = [*wxyz[1:], wxyz[0]]
        rot_phone = R.from_quat(xyzw).as_matrix()

        # Calculate new targets
        xyz_target = xyz_home + rot_phone_init.T @ (xyz_scale * (xyz_phone - xyz_phone_init))
        rot_target = rot_phone_init.T @ rot_phone @ rot_home

        # Calculate new arm joint angles
        target_joints = arm.ik_target_xyz_so3(arm.last_feedback.position, xyz_target, rot_target)

        # Set and send new goal to the arm
        goal.clear()
        goal.add_waypoint(position=target_joints, t=float(example_config.user_data['delay_time']))
        arm.set_goal(goal)

    arm.send()

mobile_io.set_led_color("red")