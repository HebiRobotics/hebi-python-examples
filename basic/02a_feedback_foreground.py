#!/usr/bin/env python3

import hebi
from time import sleep

# Get a group
lookup = hebi.Lookup()
sleep(2)

group = lookup.get_group_from_names(['family'], ['base', 'shoulder', 'elbow'])

if group == None:
  print('Group not found!')
  exit(1)

# This is by default "100" - setting this to 5 here allows the console output
# to be more reasonable.
group.feedback_frequency = 5.0

# Retrieve feedback with a blocking call to "get_next_feedback". This
# constrains the loop to run at the feedback frequency above; we run for
# about 10 seconds here

group_fbk = hebi.GroupFeedback(group.size)

for i in range(0, 50):
  # Pass in `group_fbk` to reuse the GroupFeedback instance
  group_fbk = group.get_next_feedback(group_fbk)  
  if group_fbk != None:
    print('Received feedback. Positions:\n{0}'.format(group_fbk.position))
