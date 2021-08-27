#!/usr/bin/env python3

import hebi
import numpy as np
from time import sleep
from math import sin

lookup = hebi.Lookup()
group = lookup.get_group_from_names(['X5-4'], ['X5-0000'])

if group is None:
    print('Group not found: Did you forget to set the module family and names above?')
    exit(1)

command = hebi.GroupCommand(group.size)

# Send commands to the group in a loop.
# Note that these packets may be dropped if network traffic is too high, so
# be sure to close a feedback loop at the high level!
period = 0.5
t = 0.0

positions = np.empty(group.size, dtype=np.float64)
while t < 10.0:
    for module_index in range(0, group.size):
        positions[module_index] = sin((t * 0.5) + (module_index * 0.25))

    command.position = positions
    group.send_command(command)
    sleep(period)
    t = t + period

# For critical packets, we can verify that they were sent by requesting
# confirmation from the group.  If the acknowledgement function returns
# 'True', it got positive confirmation. If it returns 'False', EITHER:
# - the sent command was dropped
# - the sent command was received by the group, but its response was either
#   dropped or the timeout period expired before receipt of the group.
# Again, a high-level process should intelligently handle these conditions!
# Note that this is a blocking call, and so for high-frequency applications,
# send_command should be used instead.
timeout_ms = 100

while t < 10.0:
    for module_index in range(0, group.size):
        positions[module_index] = sin((t * 0.5) + (module_index * 0.25))

    command.position = positions
    if group.send_command_with_acknowledgement(command):
        print('Got acknowledgement.')
    else:
        print('Did not receive acknowledgement!')

    sleep(period)
    t = t + period
