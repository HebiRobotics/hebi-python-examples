#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name  = "Test Family"
module_names = ["Actuator1", "Actuator2", "Actuator3"]

group = lookup.get_group_from_names([family_name], module_names)

if group is None:
  print('Group not found! Check that the names and families given in the source file')
  print('match modules available on the network.')
  exit(1)

print('Found group on network with {0} modules.'.format(group.size))
