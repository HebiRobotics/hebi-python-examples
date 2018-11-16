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

# Get the gains that are currently active on the actuator and save them
group_info = group.request_info()
if group_info is None:
  print('Did not receive gains from the module.')

# Save gains to a file. If this doesn't succeed, it probably indicates the directory doesn't exist.
if group_info.write_gains('gains/my_actuator_gains.xml'):
  print('Successfully read gains from module and wrote to file.')
else:
  print('Could not write gains to file.')

