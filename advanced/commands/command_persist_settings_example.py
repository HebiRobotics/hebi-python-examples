#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()
group = lookup.get_group_from_names(['X5-4'], ['X5-0000'])

if group == None:
  print('No group found!')
  exit(1)

new_name = raw_input('Enter a new name for this module:')

command = hebi.GroupCommand(group.size)

# Note that you **must** pass a list as the value here, as opposed to just a string.
# This is intentional; some fields (e.g., "name" field) in a group command
# cannot be set with a scalar value.
# The reasoning behind this is to prevent a user from accidentally setting
# all modules in a group the same name.
# 
# If you try to set "name" with a scalar value (e.g., `"foo"` instead of `["foo"]`)
# an exception will be raised, informing you that you cannot broadcast a
# scalar value to this field in all modules of the group.
command.name = [ new_name ]

# Mark that settings should be persisted across reboots
command.save_current_settings = True

print('Setting and saving new module name. Restart and verify name has been reset.')

if group.send_command_with_acknowledgement(command, 100):
  print('Got acknowledgement.')
else:
  print('Did not receive acknowledgement!')
