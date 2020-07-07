#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_names = ["J1_base", "J2_shoulder", "J3_elbow"]

group = lookup.get_group_from_names([family_name], module_names)

if group is None:
  print('Group not found: Did you forget to set the module family and name above?')
  exit(1)

try:
  model = hebi.robot_model.import_from_hrdf("hrdf/A-2085-03.hrdf")
except:
  print("Could not load HRDF.")
  exit(1)

target_xyz = [0.4, 0.0, 0.2]

################################################################
# Get position feedback from robot to use as initial conditions for local optimization.
################################################################

group_fbk = hebi.GroupFeedback(group.size)
if group.get_next_feedback(reuse_fbk=group_fbk) is None:
  print("Couldn't get feedback.")
  exit(1)

# Note: user should check if the positions are appropriate for initial conditions.
initial_joint_angles = group_fbk.position

################################################################
# Get IK Solution with one objective
################################################################

# Just one objective:
# Note: this is a numerical optimization and can be significantly affected by initial conditions (seed joint angles)
ee_pos_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
ik_result_joint_angles = model.solve_inverse_kinematics(initial_joint_angles, ee_pos_objective)

print('Target position: {0}'.format(target_xyz))
print('IK joint angles: {0}'.format(ik_result_joint_angles))
print('FK of IK joint angles: {0}'.format(model.get_end_effector(ik_result_joint_angles)[0:3, 3]))

################################################################
# Send commands to the physical robot
################################################################

# Move the arm
# Note: you could use the Hebi Trajectory API to do this smoothly
group_command = hebi.GroupCommand(group.size)
group_command.position = ik_result_joint_angles

for i in range(100):
  group.send_command(group_command)
  # Note: the arm will go limp after the 100 ms command lifetime,
  # so the command is repeated every 50 ms for 5 seconds.
  sleep(0.05)
