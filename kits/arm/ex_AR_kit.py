#!/usr/bin/env python3

import hebi
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api


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
sleep(2)

# Setup MobileIO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, arm_family, phone_name)

if m is None:
    raise RuntimeError("Could not find Mobile IO device")
m.update()
m.send_layout('./layouts/ex_AR_kit.json')

# Demo Variables
abort_flag = False
run_mode = "softstart"
goal = arm_api.Goal(arm.size)
home_position = [0, np.pi / 3, 2 * np.pi / 3, 5 * np.pi / 6, -np.pi / 2, 0]

# Command the softstart to the home position
softstart = arm_api.Goal(arm.size)
softstart.add_waypoint(t=4, position=home_position)
arm.update()
arm.set_goal(softstart)
arm.send()

# Get the cartesian position and rotation matrix @ home position
xyz_home = np.zeros(3)
rot_home = np.zeros((3, 3))
# arm.FK(home_position, xyz_home, ori_home)
arm.FK(home_position, xyz_out=xyz_home, orientation_out=rot_home)

# Get the states for the mobile device
xyz_phone_init = np.zeros(3)
rot_phone_init = np.zeros((3, 3))

# # Target variables
# target_joints = np.zeros(arm.size)

# Print Instructions
instructions = (
    "Mode: {}\n"
    "B1: Return to Home Position\n"
    "B3: Start AR Control\n"
    "B6: Grav Comp Mode\n"
    "B8: Quit")
print(instructions.format(run_mode))

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
        # <-- insert xyz_scale here if wanted -->
        xyz_target = xyz_home + rot_phone_init.T @ (0.75 * (xyz_phone - xyz_phone_init))
        rot_target = rot_phone_init.T @ rot_phone @ rot_home

        # Calculate new arm joint angles
        target_joints = arm.ik_target_xyz_so3(arm.last_feedback.position, xyz_target, rot_target)

        # Set and send new goal to the arm
        goal.clear()
        goal.add_waypoint(position=target_joints)
        arm.set_goal(goal)

    arm.send()
