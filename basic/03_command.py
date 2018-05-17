#!/usr/bin/env python3

from hebi import *
from time import sleep

# Get a group
lookup = Lookup()
sleep(2)

group = lookup.get_group_from_names(['family'], ['base', 'shoulder', 'elbow'])

if not group:
  print('Group not found!')
  exit(1)

# Sets the command lifetime to 100 milliseconds
group.command_lifetime = 100

# Nm/rad
spring_constant = -10.0
group_command = GroupCommand(group.size)

# Add a callback function to respond to feedback with a "virtual spring" command
def feedback_handler(group_fbk):
  # Apply Hooke's law: F = -k * x
  group_command.effort = spring_constant * group_fbk.position
  group.send_command(group_command)


group.add_feedback_handler(feedback_handler)

# Control the robot at 100Hz for 30 seconds
group.feedback_frequency = 100.0
sleep(30)
group.clear_feedback_handlers()
