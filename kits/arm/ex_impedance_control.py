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

arm = setup_arm(arm_name, arm_family, has_gas_spring)
group = arm.group

gravity_vec = params.gravity_vec
local_dir = params.local_dir

# Increase feedback frequency since we're calculating velocities at the
# high level for damping. Going faster can help reduce a little bit of
# jitter for fast motions, but going slower (100 Hz) also works just fine
# for most applications.
group.feedback_frequency = 200.0

# Double the effort gains from their default values, to make the arm more sensitive for tracking force.
gains = arm.gains
gains.effort_kp = 2.0 * gains.effort_kp
group.send_command(gains)

enable_logging = True

# Start background logging 
if enable_logging:
  log_file_dir = group.startLog('dir', os.path.join(local_dir, 'logs')) 

# Gravity compensated mode
cmd = hebi.GroupCommand(group.size)
fbk = hebi.GroupFeedback(group.size)

disp('Commanded gravity-compensated zero force to the arm.')
disp('  SPACE - Toggles an impedance controller on/off:')
disp('          ON  - Apply controller based on current position')
disp('          OFF - Go back to gravity-compensated mode')
disp('  ESC - Exits the demo.')

gains_in_end_effector_frame = True
damper_gains = np.asarray([10, 10, 0, .1, .1, .1])
spring_gains = np.asarray([500, 500, 0, 5, 5, 5])

fbk = group.get_next_feedback(reuse_fbk=fbk)
arm_tip_fk = arm.get_fk('endeffector', fbk.position)
end_effector_XYZ = arm_tip_fk[0:3,3]
end_effector_RotMat = arm_tip_fk[0:3,0:3]
    
controller_on = False

# Velocity commands
arm_cmd_joint_angs = fbk.position
arm_cmd_joint_vels = np.zeros(1, numDoF)


while not has_esc_been_pressed():   
  # Gather sensor data from the arm
  fbk = group.get_next_feedback(reuse_fbk=fbk)

  # Calculate required torques to negate gravity at current position
  cmd.effort = util.math_utils.get_grav_comp_efforts(kin, fbk.position, gravity_vec)

  if controller_on:
    # TODO
  else:
    # TODO

  # TODO: Finish

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
