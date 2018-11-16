#!/usr/bin/env python3

import hebi
from math import pi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_name = "Test Actuator"

group = lookup.create_group_from_names([family_name], [module_name])

if group is None:
  print('Group not found! Check that the family and name of a module on the network')
  print('matches what is given in the source file.')
  exit(1)

group_command = hebi.GroupCommand(group.size)
if group_command.read_gains('gains/example_gains.xml') and group.send_command_with_acknowledgement(group_command):
  print('Successfully read gains from file and sent to module.')
else:
  print('Problem reading gains from file or sending to module.')
