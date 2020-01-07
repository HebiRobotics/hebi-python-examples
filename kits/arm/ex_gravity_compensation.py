#!/usr/bin/env python3

import hebi
import os
import sys


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


from util.input import listen_for_escape_key, has_esc_been_pressed
from util.math_utils import get_grav_comp_efforts
from util.arm import setup_arm_params


# Listens for `ESC` being pressed
listen_for_escape_key()


arm_name = '6-DoF + gripper'
arm_family = 'Arm'
# If you attach a gas spring to the shoulder for extra payload, set this to True
has_gas_spring = False

group, kin, params = setup_arm_params(arm_name, arm_family, has_gas_spring)

gravity_vec = params.gravity_vec
effort_offset = params.effort_offset
local_dir = params.local_dir

enable_logging = True

# Start background logging 
if enable_logging:
  log_file_dir = group.start_log('dir', os.path.join(local_dir, 'logs'), mkdirs=True)

# Gravity compensated mode
cmd = hebi.GroupCommand(group.size)
fbk = hebi.GroupFeedback(group.size)

print('Commanded gravity-compensated zero torques to the arm.')
print('Press ESC to stop.')

while not has_esc_been_pressed():
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

  # Plot tracking / error from the joints in the arm.  Note that there
  # will not by any 'error' in tracking for position and velocity, since
  # this example only commands effort.
  hebi.util.plot_logs(hebi_log, 'position')
  hebi.util.plot_logs(hebi_log, 'velocity')
  hebi.util.plot_logs(hebi_log, 'effort')

  # Put more plotting code here
