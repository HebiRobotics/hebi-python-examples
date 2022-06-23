#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()
sleep(2)

master = lookup.get_group_from_names(['HEBI'], ['master'])
slave = lookup.get_group_from_names(['HEBI'], ['slave'])

if master is None:
    print('Master group not found: Did you forget to set the module family and names above?')
    exit(1)
elif slave is None:
    print('Slave group not found: Did you forget to set the module family and names above?')
    exit(1)
elif master.size != slave.size:
    print('Groups must be same size for master/slave control')
    exit(1)

command = hebi.GroupCommand(slave.size)


def feedback_handler(group_fbk):
    command.position = group_fbk.position
    slave.send_command(command)


# Start feedback callbacks
master.add_feedback_handler(feedback_handler)
master.feedback_frequency = 200.0

sleep(20)

# Stop the async callback before returning and deleting objects.
master.feedback_frequency = 0.0
master.clear_feedback_handlers()
