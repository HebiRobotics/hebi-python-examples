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


def feedback_handler(group_fbk):
    angles = group_fbk.position
    transform = model.get_end_effector(angles)
    print('x,y,z: {0}, {1}, {2}'.format(transform[0, 3], transform[1, 3], transform[2, 3]))


group.add_feedback_handler(feedback_handler)

# Control the robot at 100 Hz for 30 seconds
sleep(30)
