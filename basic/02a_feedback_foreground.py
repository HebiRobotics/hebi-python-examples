#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_name = "Test Actuator"

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
  print('Group not found: Did you forget to set the module family and name above?')
  exit(1)

# This is by default 100 Hz. Setting this to 5 Hz allows the console output to be reasonable.
group.feedback_frequency = 5.0

group_feedback = hebi.GroupFeedback(group.size)

for i in range(50):
  if group.get_next_feedback(reuse_fbk=group_feedback) is not None:
    print('Feedback received. Positions are:\n{0}'.format(group_feedback.position))
