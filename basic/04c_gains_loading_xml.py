#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_name = "Test Actuator"

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
  print('Group not found! Check that the family and name of a module on the network')
  print('matches what is given in the source file.')
  exit(1)

group_command = hebi.GroupCommand(group.size)
try:
  group_command.read_gains('gains/example_gains.xml')
  if not group.send_command_with_acknowledgement(group_command):
    raise RuntimeError('Did not receive ack from group.')
  print('Successfully read gains from file and sent to module.')
except Exception as e:
  print('Problem reading gains from file or sending to module: {0}'.format(e))
