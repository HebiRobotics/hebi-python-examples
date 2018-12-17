#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_names = ["Base", "Shoulder", "Elbow"]

group = lookup.get_group_from_names([family_name], module_names)

if group is None:
  print('Group not found! Check that the family and name of a module on the network')
  print('matches what is given in the source file.')
  exit(1)

try:
  model = hebi.robot_model.import_from_hrdf("hrdf/3-DoF_arm_example.hrdf")
except:
  print("Could not load HRDF.")
  exit(1)


def feedback_handler(group_fbk):
  angles = group_fbk.position
  transform = model.get_end_effector(angles)
  print('x,y,z: {0}, {1}, {2}'.format(transform[0, 3], transform[1, 3], transform[2, 3]))


group.add_feedback_handler(feedback_handler)

# Control the robot at 100 Hz for 30 seconds
sleep(30)
