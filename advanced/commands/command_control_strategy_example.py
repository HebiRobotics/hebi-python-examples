#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()
group = lookup.get_group_from_names(['X5-4'], ['X5-0000'])

command = hebi.GroupCommand(group.size)
# Set the control strategy for all the modules
command.control_strategy = 'strategy2'

if group.send_command_with_acknowledgement(command, 100):
  print('Got acknowledgement')
else:
  print('Did not receive acknowledgement!')
