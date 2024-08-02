#!/usr/bin/env python3

import hebi
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
from hebi.util import create_mobile_io

lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/examples/ex_AR_kit.cfg"

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
run_mode = "softstart"
goal = hebi.arm.Goal(arm.size)

# Command the softstart to the home position
softstart = hebi.arm.Goal(arm.size)
softstart.add_waypoint(t=example_config.user_data['soft_start_time'], 
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

#######################
## Main Control Loop ##
#######################

while not abort_flag:
    arm.update()  # update the arm

    if not m.update():
        print("Failed to get feedback from MobileIO")
        continue

    if run_mode == "softstart":
        # End softstart when the arm reaches the home_position
        if arm.at_goal:
            m.set_led_color("yellow")
            run_mode = "waiting"
            continue
        arm.send()
        continue

    # B1 - Return to home position
    if m.get_button_diff(1) == 1:  # "ToOn"
        m.set_led_color("yellow")
        run_mode = "waiting"
        arm.set_goal(softstart)

    # B3 - Start AR Control
    if m.get_button_diff(3) == 1 and run_mode != "ar_mode":  # "ToOn"
        m.set_led_color("green")
        run_mode = "ar_mode"

        # Store initial position and orientation as baseline
        xyz_phone_init = m.position.copy()
        wxyz = m.orientation
        xyzw = [*wxyz[1:], wxyz[0]]
        rot_phone_init = R.from_quat(xyzw).as_matrix()

    # B6 - Grav Comp Mode
    if m.get_button_diff(6) == 1:  # "ToOn"
        m.set_led_color("blue")
        run_mode = "grav_comp"
        arm.cancel_goal()

    # B8 - Quit
    if m.get_button_diff(8) == 1:  # "ToOn"
        m.set_led_color("transparent")
        abort_flag = True
        break

    if run_mode == "ar_mode":
        # Get the latest mobile position and orientation
        xyz_phone = m.position
        wxyz = m.orientation
        xyzw = [*wxyz[1:], wxyz[0]]
        rot_phone = R.from_quat(xyzw).as_matrix()

        # Calculate new targets
        xyz_target = xyz_home + rot_phone_init.T @ (example_config.user_data['xyz_scale'] * (xyz_phone - xyz_phone_init))
        rot_target = rot_phone_init.T @ rot_phone @ rot_home

        # Calculate new arm joint angles
        target_joints = arm.ik_target_xyz_so3(arm.last_feedback.position, xyz_target, rot_target)

        # Set and send new goal to the arm
        goal.clear()
        goal.add_waypoint(position=target_joints)
        arm.set_goal(goal)

    arm.send()
