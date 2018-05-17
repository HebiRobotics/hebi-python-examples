#!/usr/bin/env python3

import hebi

lookup = hebi.Lookup()
group = lookup.get_group_from_names(['family'], ['base', 'shoulder', 'elbow'])

command = hebi.GroupCommand(group.size)

# Set gains. If this doesn't succeed, it may be because the number of modules
# in the group doesn't match the number in the XML, or the file was corrupt.

try:
  command.read_gains('gains.xml')
  print('Successfully read gains from file; now sending to module.')
  group.send_command(command)
except:
  print('Could not read gains from file')
