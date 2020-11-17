#!/usr/bin/env python3

import hebi
import numpy as np
from hebi import arm as arm_api
from matplotlib import pyplot as plt

# Arm setup
arm_family   = "Arm"
module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
hrdf_file    = "hrdf/A-2085-06.hrdf"
gains_file   = "gains/A-2085-06.xml"


# Create Arm object
arm = arm_api.create([arm_family],
                     names=module_names,
                     hrdf_file=hrdf_file)
arm.load_gains(gains_file)


# Start background logging 
enable_logging = True
if enable_logging:
  arm.group.start_log('dir', 'logs', mkdirs=True)

#######################
## Main Control Loop ##
#######################

<<<<<<< HEAD
print('Commanding gravity-compensated zero torques to the arm.')
while arm.update():
  # When no goal is set, the arm automatically returns to grav-comp mode
  # Thus, when we have an empty control loop, the arm is in grav-comp
  # awaiting further instructions
=======
while not m.get_button_state(1):

  if not arm.update():
    print("Failed to update arm")
    continue

  m.update(timeout_ms=0):
>>>>>>> dac822b471caf75859ebd1cb86f20335ccebf8cc

  # Send the latest loaded commands to the arm. If no changes are made, 
  # it will send the last loaded command when arm.update() was last called
  arm.send()

##########################
## Logging and Plotting ##
##########################

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
  # Insert additional plotting code here
