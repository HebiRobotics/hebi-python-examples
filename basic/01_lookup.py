#!/usr/bin/env python3

import hebi
from time import sleep

# Create lookup
lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate, and print out its contents
sleep(2)

for entry in lookup.entrylist:
  print(entry)

# Create a group
group = lookup.get_group_from_names(['family'], ['base', 'shoulder', 'elbow'])

if group == None:
  print('Group not found')
  exit(1)

print('Found group on network: size {0}'.format(group.size))