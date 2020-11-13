#!/usr/bin/env python3

from time import sleep

import hebi
import numpy as np


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

# Example parameters
enable_logging = True
xyz_target_init = np.asarray([0.5, 0.0, 0.1])
ik_seed_pos = np.asarray([0.01, 1.0, 2.5, 1.5, -1.5, 0.01])
rot_mat_target_init = np.asarray([ # Ry(pi)
  [-1, 0, 0],
  [0, 1, 0],
  [0, 0, -1]
])

# Lookup initialization
lookup = hebi.Lookup()
sleep(2)

# Setup Mobile IO input device
phone = hebi.util.create_mobile_io(
  lookup=lookup,
  family="HEBI",
  name="mobileIO")

if phone is None:
  raise RuntimeError("Could not find Mobile IO device")
phone.update()

# Setup Arm
arm = hebi.arm.create(
  lookup=lookup,
  families=["Arm"],
  names=['J1_base',
         'J2_shoulder',
         'J3_elbow',
         'J4_wrist1',
         'J5_wrist2',
         'J6_wrist3'],
  hrdf_file="hrdf/A-2085-06.hrdf")

print("""
Arm end-effector is now following the mobile device pose.
The control interface has the following commands:
B1 - Reset/re-align poses.
     This takes the arm to home and aligns with mobile device.
A3 - Scale down the translation commands to the arm.
     Sliding all the way down means the end-effector only rotates.
A6 - Control the gripper (if the arm has gripper).
     Sliding down closes the gripper, sliding up opens.
B8 - Quits the demo.
""")

# Start background logging
if enable_logging:
  arm.group.start_log(
    directory='logs',
    name='logFile',
    mkdirs=True)

# move to initial position
goal = hebi.arm.Goal(arm.size)
zeros = np.zeros(arm.size)
xyz_target = xyz_target_init
ik_pos = arm.ik_target_xyz_so3(ik_seed_pos, xyz_target, rot_mat_target_init)
goal.add_waypoint(t=5, position=ik_pos,velocity=zeros,acceleration=zeros)
arm.set_goal(goal)
while not arm.at_goal:
  arm.update()
  arm.send()

keep_running = True
pending_goal = False
run_mode = "startup"
control_mode_toggle = 1
quit_demo_button = 8

mobile_pos_offset = [0, 0, 0]
while True:
  arm.update()
  arm.send()

  # human interface state
  if phone.update(timeout_ms=0):

    # B8 quit example
    if phone.get_button_state(8):
      break

    # B1 reset/re-align poses
    if phone.get_button_diff(1) == 3:  # "ToOn"
      break

    # A3 scale down translation commands. All the way down (-1) means only rotation
    if phone.get_axis_state(3) == 3:  # "ToOn"
      break

    # A6 open/close the gripper, if exists
    if phone.get_axis_state(6) == 3:  # "ToOn"
      break

    #xyz_target = phone.position
    ik_pos = arm.ik_target_xyz_so3(
      initial_position=arm.last_feedback.position,
      target_xyz=xyz_target,
      orientation=quat2rotMat(phone.orientation)
    )
    arm.set_goal(goal.clear().add_waypoint(t=0.1, position=ik_pos, velocity=zeros, acceleration=zeros))

print('Stopped example')
if enable_logging:
  log_file = arm.group.stop_log()
  hebi.util.plot_logs(log_file, 'position', figure_spec=101)
  hebi.util.plot_logs(log_file, 'velocity', figure_spec=102)
  hebi.util.plot_logs(log_file, 'effort', figure_spec=103)
#
#
# def get_mobile_state(quit_demo_button):
#   """
#   Mobile io function to be run in another thread to prevent main loop stalling on long feedbacks
#   """
#   global fbk_mobile
#   global keep_running
#   global run_mode
#   global mobile_pos_offset
#
#   phone.update()
#
#   phone.set_led_color("yellow")
#   while not phone.get_button_diff(quit_demo_button) == 3: # "ToOn"
#
#     if not phone.update():
#       print("Failed to get feedback from MobileIO")
#       continue
#
#     fbk_mobile = phone.get_last_feedback()
#     # Check for button presses and control state accordingly
#     if phone.get_button_diff(control_mode_toggle) == 2: # "ToOff"
#       run_mode = "startup"
#       phone.set_led_color("yellow")
#     if run_mode == "standby":
#       phone.set_led_color("green")
#     if phone.get_button_diff(control_mode_toggle) == 3 and run_mode == "standby": # "ToOn"
#       phone.set_led_color("blue")
#       run_mode = "control"
#       mobile_pos_offset = xyz_target_init - fbk_mobile.ar_position[0]
#
#   phone.set_led_color("red")
#   keep_running = False
#
#
# # Start mobile io thread
# t1 = threading.Thread(target=get_mobile_state, args=(quit_demo_button,), daemon=True)
# t1.start()
#
# print('Arm end-effector is now following the mobile device pose.')
# print('The control interface has the following commands:')
# print('  B1 - Reset/re-align poses.')
# print('       This takes the arm to home and aligns with mobile device.')
# print('  B8 - Quits the demo.')
#
#
# # Main run loop
# while keep_running:
#
#   if not arm.update():
#     print("Failed to update arm")
#     continue
#
#   if run_mode == "startup":
#     # Move to starting pos
#     joint_targets = arm.ik_target_xyz(ik_seed_pos, xyz_target_init)
#     arm.set_goal(goal.clear().add_waypoint(position=joint_targets))
#     run_mode = "moving to start pos"
#   elif run_mode == "moving to start pos":
#     # When at startup pos, switch to standby mode
#     if arm.at_goal:
#       run_mode = "standby"
#   elif run_mode == "standby":
#     # Wait for mobile io input while holding arm in place
#     pass
#   elif run_mode == "control":
#     # Follow phone's motion in 3D space
#     phone_target_xyz = fbk_mobile.ar_position[0] + mobile_pos_offset
#     joint_targets = arm.ik_target_xyz(arm.last_feedback.position, phone_target_xyz)
#     arm.set_goal(goal.clear().add_waypoint(t=1.0, position=joint_targets))
#
#   arm.send()
