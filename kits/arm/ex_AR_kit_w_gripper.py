#!/usr/bin/env python3

import hebi
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
from hebi_util import create_mobile_io_from_config, create_gripper_from_config


# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/ex_AR_kit_w_gripper.cfg.yaml"
example_config = hebi.config.load_config(example_config_file)

# Set up arm, mobile_io, and gripper from config
arm = hebi.arm.create_from_config(example_config, lookup)
mobile_io = create_mobile_io_from_config(example_config, lookup)
gripper = create_gripper_from_config(example_config, lookup, arm)

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

# # Target variables
# target_joints = np.zeros(arm.size)

# Print instructions
instructions = """Mode:

    🤌 - Gripper Control
    🏠 - Home
    📲 - AR Control
    🌍 - Grav Comp
    ❌ - Quit")"""

print(instructions)

#######################
## Main Control Loop ##
#######################

while not abort_flag:
    arm.update()  # update the arm

    if not mobile_io.update():
        print("Failed to get feedback from MobileIO")
        continue

    if run_mode == "softstart":
        # End softstart when the arm reaches the home_position
        if arm.at_goal:
            mobile_io.set_led_color("yellow")
            run_mode = "waiting"
            mobile_io.clear_text()
            mobile_io.add_text(instructions.format(run_mode))
            continue
        arm.send()
        continue

    # B1 - Return to home position
    if mobile_io.get_button_diff(1) == 1:  # Edge Down
        mobile_io.set_led_color("yellow")
        run_mode = "waiting"
        mobile_io.clear_text()
        mobile_io.add_text(instructions.format(run_mode))
        arm.set_goal(softstart)

    # B3 - Start AR Control
    if mobile_io.get_button_diff(3) == 1 and run_mode != "ar_mode":
        mobile_io.set_led_color("green")
        run_mode = "ar_mode"
        mobile_io.clear_text()
        mobile_io.add_text(instructions.format(run_mode))
        xyz_phone_init = mobile_io.position.copy()
        wxyz = mobile_io.orientation
        xyzw = [*wxyz[1:], wxyz[0]]
        rot_phone_init = R.from_quat(xyzw).as_matrix()

    # B6 - Grav Comp Mode
    if mobile_io.get_button_diff(6) == 1:
        mobile_io.set_led_color("blue")
        run_mode = "grav_comp"
        mobile_io.clear_text()
        mobile_io.add_text(instructions.format(run_mode))
        arm.cancel_goal()

    # B8 - Quit
    if mobile_io.get_button_diff(8) == 1:
        mobile_io.set_led_color("transparent")
        abort_flag = True
        break

    if run_mode == "ar_mode":
        # Get the latest mobile position and orientation
        xyz_phone = mobile_io.position
        wxyz = mobile_io.orientation
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

        # Set the gripper separataly to follow slider A3
        slider3 = mobile_io.get_axis_state(3)
        # Map slider range -1 to 1 onto close and open effort for gripper
        grip_effort = (slider3 + 1.0) / 2.0
        if arm.end_effector is not None:
            arm.end_effector.update(grip_effort)

    arm.send()
