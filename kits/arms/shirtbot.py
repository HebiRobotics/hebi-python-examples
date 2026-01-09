#!/usr/bin/env python3

import os
import hebi
from time import time, sleep
from hebi_util import create_mobile_io_from_config, create_gripper_from_config
import numpy as np

# Set up to find actuators on the network
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/open_house_demo.cfg.yaml"
example_config = hebi.config.load_config(example_config_file)

if example_config.user_data is None:
    raise RuntimeError('This example requires user_data section of config to be populated. The loaded config does not have user_data.')

# Set up arm, mobile_io, and gripper from config
arm = hebi.arm.create_from_config(example_config, lookup)

# Get initial feedback
while not arm.update():
    continue

mobile_io = create_mobile_io_from_config(example_config, lookup)
gripper = create_gripper_from_config(example_config, lookup, arm)

# Demo Variables
abort_flag = False
pending_goal = False
run_mode = "training"
goal = hebi.arm.Goal(arm.size)
base_travel_time = example_config.user_data['base_travel_time']
min_travel_time = example_config.user_data['min_travel_time']

home_position = [0, -0.6, -1.6, 1.83, -2.2, 4.73, 0]

shirt_seed = [1.42, -0.62, -1.89, 2.12, -0.3, 1.75, -0.07]

shirt_rot = np.zeros((3, 3))
shirt_rot[0,0] = -1
shirt_rot[1,1] = 1
shirt_rot[2,2] = -1

z_low_1 = 0.00
z_low_2 = 0.01
z_low_3 = 0.02
z_high = 0.28

col_1 = 0.25
col_2 = 0.55
col_3 = 0.85

front_row_x = 0.34
shirt_xs_approach = arm.ik_target_xyz_so3(shirt_seed, [front_row_x, col_1, z_high], shirt_rot)
shirt_xs = arm.ik_target_xyz_so3(shirt_xs_approach, [front_row_x, col_1, z_low_1], shirt_rot)
shirt_s_approach = arm.ik_target_xyz_so3(shirt_seed, [front_row_x, col_2, z_high], shirt_rot)
shirt_s = arm.ik_target_xyz_so3(shirt_s_approach, [front_row_x, col_2, z_low_2], shirt_rot)
shirt_m_approach = arm.ik_target_xyz_so3(shirt_seed, [front_row_x, col_3, z_high], shirt_rot)
shirt_m = arm.ik_target_xyz_so3(shirt_m_approach, [front_row_x, col_3, z_low_3], shirt_rot)

back_row_x = 0.02
shirt_l_approach = arm.ik_target_xyz_so3(shirt_seed, [back_row_x, col_1, z_high], shirt_rot)
shirt_l = arm.ik_target_xyz_so3(shirt_l_approach, [back_row_x, col_1, z_low_1], shirt_rot)
shirt_xl_approach = arm.ik_target_xyz_so3(shirt_seed, [back_row_x, col_2, z_high], shirt_rot)
shirt_xl = arm.ik_target_xyz_so3(shirt_xl_approach, [back_row_x, col_2, z_low_2], shirt_rot)
shirt_xxl_approach = arm.ik_target_xyz_so3(shirt_seed, [back_row_x, col_3, z_high], shirt_rot)
shirt_xxl = arm.ik_target_xyz_so3(shirt_xxl_approach, [back_row_x, col_3, z_low_3], shirt_rot)

goal.add_waypoint(t = 5, 
                  position=home_position, velocity=[0] * arm.size)
arm.set_goal(goal)
gripper.open()
gripper.send()

# Go home:
while not arm.at_goal:
    arm.update()
    arm.send()
    gripper.send()

#######################
## Main Control Loop ##
#######################

while not abort_flag:
    arm.update()
    arm.send()

    shirt_waypoint = None
    shirt_approach_waypoint = None
    if mobile_io.update(0.0):
      if mobile_io.get_button_diff(1) == 1:  # "ToOn"
          shirt_waypoint = shirt_xs
          shirt_approach_waypoint = shirt_xs_approach
      if mobile_io.get_button_diff(2) == 1:  # "ToOn"
          shirt_waypoint = shirt_s
          shirt_approach_waypoint = shirt_s_approach
      if mobile_io.get_button_diff(3) == 1:  # "ToOn"
          shirt_waypoint = shirt_m
          shirt_approach_waypoint = shirt_m_approach
      if mobile_io.get_button_diff(4) == 1:  # "ToOn"
          shirt_waypoint = shirt_l
          shirt_approach_waypoint = shirt_l_approach
      if mobile_io.get_button_diff(5) == 1:  # "ToOn"
          shirt_waypoint = shirt_xl
          shirt_approach_waypoint = shirt_xl_approach
      if mobile_io.get_button_diff(6) == 1:  # "ToOn"
          shirt_waypoint = shirt_xxl
          shirt_approach_waypoint = shirt_xxl_approach
      if mobile_io.get_button_diff(8) == 1:  # "ToOn"
          abort_flag = True

    if shirt_waypoint is None:
        continue

    # Go to t-shirt pickup point:
    goal = hebi.arm.Goal(arm.size)
    goal.add_waypoint(t = 3, 
                    position=shirt_approach_waypoint, velocity=[0] * arm.size)
    goal.add_waypoint(t = 3, 
                    position=shirt_waypoint, velocity=[0] * arm.size)
    arm.set_goal(goal)

    while not arm.at_goal:
        arm.update()
        arm.send()
        gripper.send()


    gripper.toggle()
    gripper.send()

    arm.update()
    arm.send()



    goal = hebi.arm.Goal(arm.size)
    goal.add_waypoint(t = 3, 
                    position=shirt_approach_waypoint, velocity=[0] * arm.size)
    goal.add_waypoint(t = 3, 
                    position=home_position, velocity=[0] * arm.size)
    arm.set_goal(goal)

    while not arm.at_goal:
        arm.update()
        arm.send()
        gripper.send()

    gripper.toggle()
