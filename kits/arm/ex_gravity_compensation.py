#!/usr/bin/env python3

import hebi
import numpy as np
import os
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api
from matplotlib import pyplot as plt

# Arm setup
phone_family = "HEBI"
phone_name   = "mobileIO"
arm_family   = "Example Arm"
hrdf_file    = "hrdf/A-2085-06.hrdf"

lookup = hebi.Lookup()
sleep(2)

# Setup MobileIO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.update()

# Setup arm components
arm = arm_api.create(arm_family,
                     lookup=lookup,
                     hrdf_file=hrdf_file)

# Configure arm components
# TODO

enable_logging = True

# Start background logging 
if enable_logging:
  arm.group.start_log('dir', 'logs', mkdirs=True)

print('Commanded gravity-compensated zero torques to the arm.')
print('Press b1 to stop.')

while not m.get_button_state(1):

  if not arm.update():
    print("Failed to update arm")
    continue

  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  arm.send()

if enable_logging:
  hebi_log = arm.group.stop_log()

  # Plot tracking / error from the joints in the arm.
  time = []
  position = []
  velocity = []
  effort = []
  # iterate through log
  for entry in hebi_log.feedback_iterate:
    time.append(entry.transmit_time)
    position.append(entry.position)
    velocity.append(entry.velocity)
    effort.append(entry.effort)

  # Offline Visualization
  # Plot the logged position feedback
  plt.figure(101)
  plt.plot(time, position)
  plt.title('Position')
  plt.xlabel('time (sec)')
  plt.ylabel('position (rad)')
  plt.grid(True)

  # Plot the logged velocity feedback
  plt.figure(102)
  plt.plot(time, velocity)
  plt.title('Velocity')
  plt.xlabel('time (sec)')
  plt.ylabel('velocity (rad/sec)')
  plt.grid(True)

  # Plot the logged effort feedback
  plt.figure(103)
  plt.plot(time, effort)
  plt.title('Effort')
  plt.xlabel('time (sec)')
  plt.ylabel('effort (N*m)')
  plt.grid(True)
  # Put more plotting code here
