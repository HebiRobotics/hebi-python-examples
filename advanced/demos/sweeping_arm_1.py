#!/usr/bin/env python3

# Sweeping arm demo which moves in a direction until it encounters an obstruction (sensed by reading effort feedback)

import hebi
from time import sleep, time


def is_obstructed(group_fbk, thresh, scale):
  effort = group_fbk.effort[0]
  return scale*effort > thresh


lookup = hebi.Lookup()
sleep(2)

group = lookup.get_group_from_names(['HEBI'], ['X5-1'])
cmd = hebi.GroupCommand(1)
cmd.velocity = 1.5

group.feedback_frequency = 100.0

# Used to prevent a module from getting into a situation where noise from oscillating back and forth makes
# the actuator bounce directions continuously
global last_obstruction_time
last_obstruction_time = 0.0
OBSTRUCTION_DEAD_ZONE = 0.075


def sweeping_handler(group_fbk):
  if is_obstructed(group_fbk, 0.8, cmd.velocity):
    now_time = time()

    global last_obstruction_time
    # To limit thrashing, only change directions outside of the deadzone period
    if (now_time - last_obstruction_time) > OBSTRUCTION_DEAD_ZONE:
      last_obstruction_time = time()
      cmd.velocity = -cmd.velocity

  # TODO: check if obstruction is detected
  group.send_command(cmd)

group.add_feedback_handler(sweeping_handler)

# Runs for a minute
sleep(60.0)
