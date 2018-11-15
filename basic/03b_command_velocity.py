#!/usr/bin/env python3

import hebi
from math import pi, sin
from time import sleep, time

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_name = "Test Actuator"

group = lookup.create_group_from_names([family_name], [module_name])

if group is None:
  print('Group not found! Check that the family and name of a module on the network')
  print('matches what is given in the source file.')
  exit(1)

group_command  = hebi.GroupCommand(group.size)
group_feedback = hebi.GroupFeedback(group.size)

# Start logging in the background
group.start_log('logs')

freq_hz = 0.5                 # [Hz]
freq    = freq_hz * 2.0 * pi  # [rad / sec]
amp     = 1.0                 # [rad] (45 degrees)

duration = 10.0               # [sec]
start = time()
t = time() - start

while t < duration:
  # Even though we don't use the feedback, getting feedback conveniently
  # limits the loop rate to the feedback frequency
  group.get_next_feedback(reuse_fbk=group_feedback)
  t = time() - start

  group_command.velocity = amp * sin(freq * t)
  group.send_command(group_command)

# Stop logging. `log_file` contains the contents of the file
log_file = group.stop_log()
