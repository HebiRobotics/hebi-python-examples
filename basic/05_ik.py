#!/usr/bin/env python3

from hebi import *
from math import pi
from time import sleep

# Get a group
lookup = Lookup()
sleep(2)

group = lookup.get_group_from_names(['family'], ['base', 'shoulder', 'elbow'])

if not group:
  print('Group not found!')
  exit(1)

# Create a simple kinematic description of the arm
kin = robot_model.RobotModel()
kin.add_actuator('X5-4')
kin.add_bracket('X5-LightBracket', mount='right')
kin.add_actuator('X5-4')
kin.add_link('X5', extension=0.18, twist=pi)
kin.add_actuator('X5-4')
kin.add_link('X5', extension=0.28, twist=0)

target_xyz = [0.4, 0.0, 0.2]

if not group.send_feedback_request():
  print('Could not send feedback request.')
  exit(1)

group_fbk = group.get_next_feedback()
if not group_fbk:
  print('Could not get feedback.')
  exit(1)

initial_joint_angles = group_fbk.position

# ------------------------------------------------------------------------------
# Get IK solution with one objective
# ------------------------------------------------------------------------------

# Just one objective
position_objective = robot_model.endeffector_position_objective(target_xyz)

# Note - this is a numerical optimization, and can be significantly affecting
# by initial conditions (e.g., seed joint angles)
ik_result_joint_angles = kin.solve_inverse_kinematics(initial_joint_angles,
                                                      position_objective)

transform = kin.get_end_effector(ik_result_joint_angles)

print('Target Position:\n{0}'.format(target_xyz))
print('IK joint angles:\n{0}'.format(ik_result_joint_angles))
print('FK of IK joint angles:\n{0}'.format(transform[0:3,3].A1))

# Set joint limits to force a particular solution (elbow up, in this case)
min_positions = [-pi, 0.25, 0.25]
max_positions = [pi, 1.0, 1.0]

# ------------------------------------------------------------------------------
# Get IK solution with multiple objectives
# ------------------------------------------------------------------------------

joint_limit = robot_model.joint_limit_constraint(min_positions, max_positions)
ik_result_joint_angles = kin.solve_inverse_kinematics(initial_joint_angles,
                                                      position_objective,
                                                      joint_limit)

transform = kin.get_end_effector(ik_result_joint_angles)

print('Target Position:\n{0}'.format(target_xyz))
print('IK joint angles:\n{0}'.format(ik_result_joint_angles))
print('FK of IK joint angles:\n{0}'.format(transform[0:3,3].A1))

# ------------------------------------------------------------------------------
# Send commands to the physical robot
# ------------------------------------------------------------------------------

group.command_lifetime = 100

# Move the arm (note - could use the HEBI Trajectory API to do this smoothly)
group_cmd = GroupCommand(group.size)
group_cmd.position = ik_result_joint_angles

# Note - the arm will go limp after the 100 ms command lifetime, so we repeat
# the command in a loop here until we terminate after approximately 5 seconds
for i in range(0, 100):
  group.send_command(group_cmd)
  sleep(0.05)
