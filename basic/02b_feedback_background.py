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

def feedback_handler(group_fbk):
  print('Received feedback. Positions:\n{0}'.format(group_fbk.position)) 

# Add a callback to react to feedback received on a background thread.
# This function needs to accept 1 argument, `group_fbk`
group.add_feedback_handler(feedback_handler)

# Wait for 10 seconds and stop
sleep(10)

group.clear_feedback_handlers()
