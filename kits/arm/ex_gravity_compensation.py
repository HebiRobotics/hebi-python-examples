import hebi
import os
import sys


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


from util import listen_for_escape_key, has_esc_been_pressed

# Listens for `ESC` being pressed
listen_for_escape_key()


arm_name = '6-DoF + gripper'
arm_family = 'Arm'
# If you attach a gas spring to the shoulder for extra payload, set this to True
has_gas_spring = False

group, kin, params = setup_arm(arm_name, arm_family, has_gas_spring)

gravity_vec = params.gravity_vec
effort_offset = params.effort_offset
local_dir = params.local_dir

enable_logging = True

# Start background logging 
if enable_logging:
  log_file_dir = group.startLog('dir', os.path.join(local_dir, 'logs')) 

# Gravity compensated mode
cmd = hebi.GroupCommand(group.size)
fbk = hebi.GroupFeedback(group.size)

print('Commanded gravity-compensated zero torques to the arm.')
print('Press ESC to stop.')

while not has_esc_been_pressed():   
  # Gather sensor data from the arm
  fbk = group.get_next_feedback(reuse_fbk=fbk)

  # Calculate required torques to negate gravity at current position
  cmd.effort = util.math_utils.get_grav_comp_efforts(kin, fbk.position, gravity_vec) + effort_offset

  # Send to robot
  group.send_command(cmd)

  if escape_pressed():
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
  kinematics_analysis(hebilog, kin)

  # Put more plotting code here
