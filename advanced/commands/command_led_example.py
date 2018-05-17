#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()
group = lookup.get_group_from_names(['family'], ['name'])

# Set command lifetime to 100 ms
group.command_lifetime = 100

group_command = hebi.GroupCommand(group.size)

# Set the LED red
group_command.led.color = 'red'
group.send_command(group_command)

sleep(3)

# Set the LED Blue, and then clear the command.
# Note that this "clear" does not return the LED to module control, but
# rather remove any LED command from the command object, so when this is sent
# to the module the LED state won't be affected.
group_command.led.color = 'blue'
group_command.led.color = None
group.send_command(group_command)

sleep(3)

# Set the LED to "module control" mode; the first three digits are ignored
# if the 'alpha' value is zero.
group_command.led.color = 'transparent'
group.send_command(group_command)

sleep(3)

# Set the LED purple. Note that this override automatically sets the alpha
# channel to "255" (e.g., arguments are RGB).
group_command.led.color = hebi.Color(0, 255, 255)
group.send_command(group_command)

sleep(3)
  
# Set the LED to module control.
group_command.led.color = 'transparent'
group.send_command(group_command)
