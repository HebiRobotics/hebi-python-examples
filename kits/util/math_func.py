import numpy as np
from math import cos, sin


def rotate_x(angle, dtype=np.float64):
  c = cos(angle)
  s = sin(angle)
  return np.matrix([
    [1.0, 0.0, 0.0],
    [0.0, c, -s],
    [0.0, s, c]
    ], dtype=np.float64)


def rotate_y(angle, dtype=np.float64):
  c = cos(angle)
  s = sin(angle)
  return np.matrix([
    [c, 0.0, s],
    [0.0, 1.0, 0.0],
    [-s, 0.0, c]
    ], dtype=np.float64)


def rotate_z(angle, dtype=np.float64):
  c = cos(angle)
  s = sin(angle)
  return np.matrix([
    [c, -s, 0.0],
    [s, c, 0.0],
    [0.0, 0.0, 1.0]
    ], dtype=np.float64)


def get_grav_comp_efforts(robot, positions, gravity):
  gravity = gravity / np.linalg.norm(gravity)*9.81
  jacobians = robot.get_jacobians('CoM', positions)
  comp_torque = np.zeros((robot.dof_count, 1))
  wrench = np.zeros(6)
  num_frames = robot.get_frame_count('CoM')

  masses = robot.masses

  for i in range(num_frames):
    # Add the torques for each joint to support the mass at this frame
    wrench[0:3] = gravity*masses[i]
    comp_torque += jacobians[i].T*np.reshape(wrench, (6, 1))

  return np.squeeze(comp_torque)


def get_dynamic_comp_efforts(fbk_positions, cmd_positions, cmd_velocities, cmd_accels, robot, dt=1e-3):
  dt_s = dt*dt
  dt_s_inv = 1/dt_s
  cmd_v_dt = cmd_velocities*dt
  cmd_accel_dt = .5*cmd_accels*dt_s

  # get positions at +/- dt
  cmd_positions_last = cmd_positions-cmd_v_dt+cmd_accel_dt
  cmd_positions_next = cmd_positions+cmd_v_dt+cmd_accel_dt

  # Get Forward kinematics for all 3 sets of angles
  cmd_frames_last = [entry[:3, 3] for entry in robot.get_forward_kinematics('CoM', cmd_positions_last)]
  cmd_frames_now = [entry[:3, 3] for entry in robot.get_forward_kinematics('CoM', cmd_positions)]
  cmd_frames_next = [entry[:3, 3] for entry in robot.get_forward_kinematics('CoM', cmd_positions_next)]

  # Build wrench vector and calculate compensatory torques
  efforts = np.zeros((robot.dof_count, 1))
  jacobians = robot.get_jacobians('CoM', fbk_positions)
  masses = robot.masses
  wrench = np.zeros(6)

  # used to debug the differences between the python and matlab code
  # scipy.io.savemat('testMat.mat', dict(x=jacobians, y=cmdFramesNext))

  # print robot_model_masses - robot_model.masses
  for module in range(len(masses)):
    # Calculate XYZ accelerations of the CoM
    lastv = cmd_frames_last[module]
    nowv = cmd_frames_now[module]
    nextv = cmd_frames_next[module]

    accel = ((lastv+nextv)-(2*nowv))*dt_s_inv

    # Set translational part of wrench vector (rotational stays zero)
    wrench[0:3] = accel*masses[module]

    # compEffort = J' * wrench
    efforts += jacobians[module].T*np.reshape(wrench, (6, 1))

  return np.squeeze(efforts)

