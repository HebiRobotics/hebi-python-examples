#!/usr/bin/env python3

import hebi
import os
import sys


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


from hebi.util import create_mobile_io
from util.math_utils import get_grav_comp_efforts
from util.arm import setup_arm_params
from matplotlib import pyplot as plt

# Set up our mobile io interface
phone_family = "HEBI"
phone_name = "Mobile IO"

lookup = hebi.Lookup()
sleep(2)

print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
m.update()

arm_family = 'Example Arm'
arm_name   = '6-DoF'
# If you attach a gas spring to the shoulder for extra payload, set this to True
has_gas_spring = False

group, kin, params = setup_arm_params(arm_name, arm_family, has_gas_spring)

gravity_vec = params.gravity_vec
effort_offset = params.effort_offset
local_dir = params.local_dir

enable_logging = True

# Start background logging 
if enable_logging:
  group.start_log('dir', 'logs', mkdirs=True)

# Gravity compensated mode
cmd = hebi.GroupCommand(group.size)
fbk = hebi.GroupFeedback(group.size)

print('Commanded gravity-compensated zero torques to the arm.')
print('Press b1 to stop.')

while not m.get_button_state(1):
  # Update MobileIO state
  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue
    
  # Gather sensor data from the arm
  group.get_next_feedback(reuse_fbk=fbk)

  # Update gravity vector the base module of the arm
  params.update_gravity(fbk)
  gravity_vec = params.gravity_vec

  # Calculate required torques to negate gravity at current position
  cmd.effort = get_grav_comp_efforts(kin, fbk.position, -gravity_vec) + effort_offset

  # Send to robot
  group.send_command(cmd)


if enable_logging:
  hebi_log = group.stop_log()

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
