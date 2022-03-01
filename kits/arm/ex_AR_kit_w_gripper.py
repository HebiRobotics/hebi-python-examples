#!/usr/bin/env python3

import hebi
import numpy as np
import threading
import copy
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api


# TODO: move this helper function into utils
def quat2rotMat(q):
  """
  QUAT2DCM Conversion of a quaternion to an orthogonal rotation matrix.
  Assumes that the scalar element, q_w, is the first element of the
  quaternion vector, q = [q_w q_x q_y q_z].
    R = quat2rotMat( q )
  """
  w = q[0]
  x = q[1]
  y = q[2]
  z = q[3]
  return np.asarray([
    [x * x - y * y - z * z + w * w, 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), -x * x + y * y - z * z + w * w, 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), -x * x - y * y + z * z + w * w]
  ])
# quat2rotMat test:
#print(quat2rotMat([0.77545035,  0.4112074 ,  0.24011487, -0.41464436]))
# R = [[ 0.54082968  0.84054625  0.03138466]
#      [-0.44559821  0.31795693 -0.8368664 ]
#      [-0.71340398  0.43861729  0.54650651]]


# Arm setup
arm_family   = "Arm"
module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
hrdf_file    = "hrdf/A-2085-06G.hrdf"
gains_file   = "gains/A-2085-06.xml"

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

# Add the gripper 
gripper_family = arm_family
gripper_name   = 'gripperSpool'

gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])
while gripper_group is None:
  print(f"Looking for gripper module {gripper_family} / {gripper_name} ...")
  sleep(1)
  gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])

gripper = arm_api.Gripper(gripper_group, -5, 1)
gripper.load_gains("gains/gripper_spool_gains.xml")
arm.set_end_effector(gripper)

# Demo Variables
abort_flag = False
run_mode = "softstart"
goal = arm_api.Goal(arm.size)
home_position = [0, np.pi/3, 2*np.pi/3, 5*np.pi/6, -np.pi/2, 0]

# Command the softstart to the home position
softstart = arm_api.Goal(arm.size)
softstart.add_waypoint(t=4, position=home_position)
arm.update()
arm.set_goal(softstart)
arm.send()

# Get the cartesian position and rotation matrix @ home position
xyz_home = np.zeros(3)
rot_home = np.zeros((3,3))
# arm.FK(home_position, xyz_home, ori_home)
arm.FK(home_position, xyz_out=xyz_home, orientation_out=rot_home)


# Get the states for the mobile device
xyz_phone_init = np.zeros(3)
rot_phone_init = np.zeros((3,3))

# # Target variables
# target_joints = np.zeros(arm.size)

instructions = (
        "Mode: {}\n"
        "A3: Gripper Control\n"
        "B1: Home\n"
        "B3: AR Control\n"
        "B6: Grav Comp\n"
        "B8: Quit")
m.clear_text()
m.add_text(instructions.format(run_mode))

#######################
## Main Control Loop ##
#######################


while not abort_flag:
  arm.update() # update the arm

  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  if run_mode == "softstart":
    # End softstart when the arm reaches the home_position
    if arm.at_goal:
      m.set_led_color("yellow")
      run_mode = "waiting"
      m.clear_text()
      m.add_text(instructions.format(run_mode))
      continue
    arm.send()
    continue

  # B1 - Return to home position
  if m.get_button_diff(1) == 1: # Edge Down
    m.set_led_color("yellow")
    run_mode = "waiting"
    m.clear_text()
    m.add_text(instructions.format(run_mode))
    arm.set_goal(softstart)

  # B3 - Start AR Control
  if m.get_button_diff(3) == 1 and run_mode != "ar_mode":
    m.set_led_color("green")
    run_mode = "ar_mode"
    m.clear_text()
    m.add_text(instructions.format(run_mode))
    xyz_phone_init = m.position.copy()
    rot_phone_init = quat2rotMat(m.orientation)

  # B6 - Grav Comp Mode
  if m.get_button_diff(6) == 1:
    m.set_led_color("blue")
    run_mode = "grav_comp"
    m.clear_text()
    m.add_text(instructions.format(run_mode))
    arm.cancel_goal()

  # B8 - Quit
  if m.get_button_diff(8) == 1:
    m.set_led_color("transparent")
    abort_flag = True
    break

  if run_mode == "ar_mode":
    # Get the latest mobile position and orientation
    xyz_phone = m.position
    rot_phone = quat2rotMat(m.orientation)

    # Calculate new targets
    # <-- insert xyz_scale here if wanted -->
    xyz_target = xyz_home + (0.75 * np.matmul(rot_phone_init.T, (xyz_phone - xyz_phone_init)))
    rot_target = np.matmul(np.matmul(rot_phone_init.T, rot_phone), rot_home)

    # Calculate new arm joint angles
    target_joints = arm.ik_target_xyz_so3(arm.last_feedback.position, xyz_target, rot_target)

    # Set and send new goal to the arm
    goal.clear()
    goal.add_waypoint(position=target_joints)
    arm.set_goal(goal)

    # Set the gripper separataly to follow slider A3
    slider3 = m.get_axis_state(3)
    # Map slider range -1 to 1 onto close and open effort for gripper
    grip_effort = (slider3+1)/2
    if arm.end_effector is not None:
      arm.end_effector.update(grip_effort)


  arm.send()

