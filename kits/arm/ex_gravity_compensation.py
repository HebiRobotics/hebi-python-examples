#!/usr/bin/env python3

from time import sleep, time

import hebi
from matplotlib import pyplot as plt

# Lookup initialization
lookup = hebi.Lookup()
sleep(2)

# Arm setup
arm = hebi.arm.create(
  families=["Arm"],
  names=['J1_base',
         'J2_shoulder',
         'J3_elbow',
         'J4_wrist1',
         'J5_wrist2',
         'J6_wrist3'],
  lookup=lookup,
  hrdf_file="hrdf/A-2085-06.hrdf")

# Example parameters
duration = 10  # [s]
enable_logging = True

# Start background logging
if enable_logging:
  arm.group.start_log(
    directory='logs',
    name='logFile',
    mkdirs=True)

print('Commanding gravity-compensating torques')
t0 = time()
while time() - t0 < duration:

  if not arm.update():
    print("Failed to update arm")
    continue

  arm.send()

print('Stopped commands')

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

  plt.show()
  # Put more plotting code here
