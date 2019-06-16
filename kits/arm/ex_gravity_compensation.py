import hebi
from .util import setup_arm

import os

arm_name = '6-DoF + gripper'
arm_family = 'Arm'
# If you attach a gas spring to the shoulder for extra payload, set this to True
has_gas_spring = False

group, kin, params = setup_arm( arm_name, arm_family, has_gas_spring )

gravity_vec = params.gravity_vec
effort_offset = params.effort_offset
local_dir = params.local_dir

enable_logging = true

# Start background logging 
if enable_logging:
   logFile = group.startLog('dir', os.path.join(local_dir, '/logs')) 

# Gravity compensated mode
cmd = hebi.GroupCommand(group.size)

# Keyboard input
kb = HebiKeyboard()
keys = read(kb)

print('Commanded gravity-compensated zero torques to the arm.')
print('Press ESC to stop.')

while True:   
  # Gather sensor data from the arm
  fbk = group.getNextFeedback()

  # Calculate required torques to negate gravity at current position
  cmd.effort = kin.getGravCompEfforts( fbk.position, gravity_vec ) + effort_offset

  # Send to robot
  group.send_command(cmd)

  # Check for new key presses on the keyboard
  keys = read(kb)

  if esc_pressed():
    break


if enable_logging:
  hebi_log = group.stop_log()

  # Plot tracking / error from the joints in the arm.  Note that there
  # will not by any 'error' in tracking for position and velocity, since
  # this example only commands effort.
  hebi.util.plot_logs(hebi_log, 'position')
  hebi.util.plot_logs(hebi_log, 'velocity')
  hebi.util.plot_logs(hebi_log, 'effort')

  # Plot the end-effectory trajectory and error
  kinematics_analysis( hebilog, kin )

  # Put more plotting code here
