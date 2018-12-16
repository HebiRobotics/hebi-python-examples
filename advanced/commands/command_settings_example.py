#!/usr/bin/env python3

import hebi

lookup = hebi.Lookup()
group = lookup.get_group_from_names(['X5-4'], ['X5-0000'])

if group is None:
  print('Group not found: Did you forget to set the module family and names above?')
  exit(1)

command = hebi.GroupCommand(group.size)

# Set the name for this module, based on its index within the group.
# (Note - we do not save this value, so upon module reset this will be cleared)
command.name = [str(entry) for entry in range(0, group.size)]

if group.send_command_with_acknowledgement(command, 100):
  print('Got acknowledgement.')
else:
  print('Did not receive acknowledgement!')
