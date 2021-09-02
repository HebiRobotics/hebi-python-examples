#!/usr/bin/env python3

import hebi
from time import sleep, time
from matplotlib import pyplot as plt

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_name = "Test Actuator"

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

group_command = hebi.GroupCommand(group.size)
group_feedback = hebi.GroupFeedback(group.size)

# Start logging in the background
group.start_log('logs', mkdirs=True)

print('  Move the module to make the output move...')

duration = 4  # [sec]
start = time()
t = time() - start

while t < duration:
    # Even though we don't use the feedback, getting feedback conveniently
    # limits the loop rate to the feedback frequency
    group.get_next_feedback(reuse_fbk=group_feedback)
    t = time() - start

    # Command a velocity that counters the measured angular velocity around the z-axis (same axis as the output)
    group_command.velocity = group_feedback.gyro[:, 2]
    group.send_command(group_command)

# Stop logging. `log_file` contains the contents of the file
log_file = group.stop_log()
log_file.load()

if log_file is not None:
    hebi.util.plot_logs(log_file, 'velocity', figure_spec=101)
    hebi.util.plot_logs(log_file, 'gyro', figure_spec=102)
