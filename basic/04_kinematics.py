#!/usr/bin/env python3

from hebi import *
from math import pi
from time import sleep

# Get a group
lookup = Lookup()
sleep(2)

group = lookup.get_group_from_names(['family'], ['base', 'shoulder', 'elbow'])

if not group:
  print('Group not found: Did you forget to set the module family and names above?')
  exit(1)

# Create a simple kinematic description of the arm 
kin = robot_model.RobotModel()
kin.add_actuator('X5-4')
kin.add_bracket('X5-LightBracket', mount='right')
kin.add_actuator('X5-4')
kin.add_link('X5', extension=0.18, twist=pi)
kin.add_actuator('X5-4')
kin.add_link('X5', extension=0.28, twist=0)

# Add a callback function to print (x,y,z) position
def feedback_handler(group_fbk):
  transform = kin.get_end_effector(group_fbk.position)
  print('x {0}'.format(transform.item(0, 3)))
  print('y {0}'.format(transform.item(1, 3)))
  print('z {0}'.format(transform.item(2, 3)))


group.add_feedback_handler(feedback_handler)

# Control the robot at 100Hz for 30 seconds
group.feedback_frequency = 100
sleep(30)

group.clear_feedback_handlers()
