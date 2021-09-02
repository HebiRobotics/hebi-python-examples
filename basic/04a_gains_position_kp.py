#!/usr/bin/env python3

import hebi
from math import fmod, pi
from time import sleep, time

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_name = "Test Actuator"

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

# The command struct has fields for gains in addition to the basic position, velocity, and effort commands.
gain_command = hebi.GroupCommand(group.size)
# Add a separate command object for our step command, so gains are not send in each position command.
position_command = hebi.GroupCommand(group.size)
group_feedback = hebi.GroupFeedback(group.size)

# Start logging in the background
group.start_log('logs', mkdirs=True)

# Parameters for step function
step_period = 1.0                  # sec
step_amplitude = pi / 8.0          # rad
step_duration = step_period * 4.0  # the 4.0 multiplier gives 2 full cycles

# Make the position controller gradually stiffer.
new_position_kp_gains = [0.2, 0.5, 10.0]

for new_gain in new_position_kp_gains:
    # Update command and send to the actuator
    for i in range(group.size):
        gain_command.position_kp = new_gain
    if not group.send_command_with_acknowledgement(gain_command):
        print('Did not get acknowledgement from module when sending gains. Check connection.')
        exit(1)
    print('Set position kP gain to: {0}'.format(new_gain))

    start = time()
    t = time() - start

    while t < step_duration:
        group.get_next_feedback(reuse_fbk=group_feedback)

        t = time() - start
        if fmod(t, 2.0 * step_period) > step_period:
            position = step_amplitude
        else:
            position = 0.0

        position_command.position = position

        group.send_command(position_command)

log_file = group.stop_log()
if log_file is not None:
    log_file.load()
    hebi.util.plot_logs(log_file, 'position', figure_spec=101)
