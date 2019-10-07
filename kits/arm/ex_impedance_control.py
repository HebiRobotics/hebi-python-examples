#!/usr/bin/env python3

import hebi
import numpy as np
import os
import sys

# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


from util.input import listen_for_escape_key, has_esc_been_pressed, listen_for_space_bar, has_space_been_pressed, clear_esc_state, clear_space_state
from util.math_utils import get_grav_comp_efforts, rot2axisangle
from util.arm import setup_arm_params


# Listens for `ESC`, `SPACE` being pressed
listen_for_escape_key()
listen_for_space_bar()


arm_name = '6-DoF + gripper'
arm_family = 'Arm'
# If you attach a gas spring to the shoulder for extra payload, set this to True
has_gas_spring = False

group, kin, params = setup_arm_params(arm_name, arm_family, has_gas_spring)

gravity_vec = params.gravity_vec.copy()
effort_offset = params.effort_offset
local_dir = params.local_dir

# Increase feedback frequency since we're calculating velocities at the
# high level for damping. Going faster can help reduce a little bit of
# jitter for fast motions, but going slower (100 Hz) also works just fine
# for most applications.
group.feedback_frequency = 200.0

# Double the effort gains from their default values, to make the arm more sensitive for tracking force.
gains = params.gains
gains.effort_kp = 2.0 * gains.effort_kp
group.send_command(gains)

enable_logging = True

# Start background logging 
if enable_logging:
  log_file_dir = group.start_log('dir', os.path.join(local_dir, 'logs')) 

# Gravity compensated mode
cmd = hebi.GroupCommand(group.size)
fbk = hebi.GroupFeedback(group.size)

print('Commanded gravity-compensated zero force to the arm.')
print('  SPACE - Toggles an impedance controller on/off:')
print('          ON  - Apply controller based on current position')
print('          OFF - Go back to gravity-compensated mode')
print('  ESC - Exits the demo.')

gains_in_end_effector_frame = True
damper_gains = np.asarray([10, 10, 0, .1, .1, .1])
spring_gains = np.asarray([500, 500, 0, 5, 5, 5])

fbk = group.get_next_feedback(reuse_fbk=fbk)

# FIXME: Fix the hellish broadcasting shape issues

# Error terms
pos_error = np.zeros(fbk.size)
vel_error = np.zeros(fbk.size)

xyz_error = np.zeros((3, 1))
rot_error_vec = np.zeros((3, 1))

spring_wrench = np.zeros((6, 1))
damper_wrench = np.zeros((6, 1))

# Effort components
impedance_effort = np.zeros((6, 1))
grav_effort = np.zeros((6, 1))

# Velocity commands
arm_cmd_joint_angs = fbk.position
arm_cmd_joint_vels = np.zeros((1, fbk.size))

arm_tip_fk = kin.get_end_effector(arm_cmd_joint_angs)
J_arm_tip = kin.get_jacobian_end_effector(arm_cmd_joint_angs)

end_effector_XYZ = arm_tip_fk[0:3,3].copy()
end_effector_rot_mat = arm_tip_fk[0:3,0:3].copy()


controller_on = False


while not has_esc_been_pressed():
  # Gather sensor data from the arm
  fbk = group.get_next_feedback(reuse_fbk=fbk)

  fbk_position = fbk.position

  # Update gravity vector the base module of the arm
  params.update_gravity(fbk)
  gravity_vec[:] = -params.gravity_vec

  # Calculate required torques to negate gravity at current position
  grav_effort = get_grav_comp_efforts(kin, fbk_position, gravity_vec) + effort_offset

  if controller_on:
    # Get Updated Forward Kinematics and Jacobians
    kin.get_end_effector(fbk_position, output=arm_tip_fk)
    kin.get_jacobian_end_effector(fbk_position, output=J_arm_tip)

    # Calculate Impedence Control Wrenches and Appropraite Joint Torque
    spring_wrench[:] = 0
    damper_wrench[:] = 0

    # Linear error is easy
    xyz_error[0:3] = (end_effector_XYZ - arm_tip_fk[0:3, 3]).reshape(3, 1)

    # Rotational error involves calculating axis-angle from the
    # resulting error in S03 and providing a torque around that axis.
    error_rot_mat = end_effector_rot_mat * arm_tip_fk[0:3, 0:3].T
    axis, angle = rot2axisangle(error_rot_mat)
    rot_error_vec[:, 0] = angle * axis

    if gains_in_end_effector_frame:
      xyz_error[0:3] = arm_tip_fk[0:3, 0:3].T * xyz_error.reshape((3, 1))
      rot_error_vec[0:3] = arm_tip_fk[0:3, 0:3].T * rot_error_vec

    pos_error[0:3] = xyz_error.flat
    pos_error[3:6] = rot_error_vec.flat
    vel_error[0:6] = (J_arm_tip * (arm_cmd_joint_vels - fbk.velocity).T).flat

    # spring_wrench = spring_gains *. pos_error
    spring_wrench[:, 0] = spring_gains * pos_error.flatten()

    if gains_in_end_effector_frame:
      spring_wrench[0:3] = arm_tip_fk[0:3, 0:3] * spring_wrench[0:3]
      spring_wrench[3:6] = arm_tip_fk[0:3, 0:3] * spring_wrench[3:6]

    # damper_wrench = damper_gains *. vel_error
    damper_wrench[:, 0] = damper_gains * vel_error.flatten()

    impedance_effort[:, 0] = (J_arm_tip.T * (spring_wrench + damper_wrench)).T
    grav_effort[:] = grav_effort + impedance_effort.flatten()
    cmd.effort = grav_effort
  else:
    cmd.effort = grav_effort

  # Send to robot
  group.send_command(cmd)

  # See if user requested mode toggle
  if has_space_been_pressed():
    # Toggle mode & clear space state
    controller_on = not controller_on
    clear_space_state()

    # Update the EE position
    end_effector_XYZ = arm_tip_fk[0:3,3].copy()
    end_effector_rot_mat = arm_tip_fk[0:3,0:3].copy()


if enable_logging:
  hebi_log = group.stop_log()

  # Plot tracking / error from the joints in the arm. Note that there
  # will not by any 'error' in tracking for position and velocity, since
  # this example only commands effort.
  hebi.util.plot_logs(hebi_log, 'position')
  hebi.util.plot_logs(hebi_log, 'velocity')
  hebi.util.plot_logs(hebi_log, 'effort')

  # Put more plotting code here
