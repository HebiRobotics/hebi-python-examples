import numpy as np
from math import atan2, cos, degrees, isnan, sin, sqrt


def sign(val):
  """
  Mimics MATLAB's sign function
  :param val:
  :return:
  """
  if val > 0.0:
    return 1
  elif val == 0.0:
    return 0.0
  else:
    return -1


def zero_on_nan(val):
  if isnan(val):
    return 0.0
  else:
    return val


def any_nan(mat):
  return np.any(np.isnan(mat))


def assert_not_nan(val, msg):
  if isnan(val):
    raise ValueError('{0} is nan'.format(msg))


def rotate_x(angle, dtype=np.float64, output=None):
  """

  :param angle:
  :param dtype:

  :param output:
  :type output: np.ndarray, NoneType

  :return: 3x3 rotation matrix
  :rtype:  np.matrix
  """
  c = cos(angle)
  s = sin(angle)
  if output is None:
    output = np.empty((3, 3), dtype=dtype)
  output[1, 1] = c
  output[2, 1] = s
  output[1, 2] = -s
  output[2, 2] = c
  output[0, 0] = 1.0
  output[1:3, 0] = output[0, 1:3] = 0.0
  return output


def rotate_y(angle, dtype=np.float64, output=None):
  """

  :param angle:
  :param dtype:

  :param output:
  :type output: np.ndarray, NoneType

  :return: 3x3 rotation matrix
  :rtype:  np.matrix
  """
  c = cos(angle)
  s = sin(angle)
  if output is None:
    output = np.empty((3, 3), dtype=dtype)
  output[0, 0] = c
  output[0, 2] = s
  output[2, 0] = -s
  output[2, 2] = c
  output[1, 1] = 1.0
  output[0, 1] = output[1, 0] = output[1, 2] = output[2, 1] = 0.0
  return output


def rotate_z(angle, dtype=np.float64, output=None):
  """

  :param angle:
  :param dtype:

  :param output:
  :type output: np.ndarray, NoneType

  :return: 3x3 rotation matrix
  :rtype:  np.matrix
  """
  c = cos(angle)
  s = sin(angle)
  if output is None:
    output = np.empty((3, 3), dtype=dtype)
  output[0, 0] = c
  output[0, 1] = s
  output[1, 0] = -s
  output[1, 1] = c
  output[2, 2] = 1.0
  output[2, 0:2] = output[0:2, 2] = 0.0
  return output


def quat2rot(quaternion, output=None):
  if output is None:
    output = np.empty((3, 3), dtype=np.float64)
  X = quaternion[1]
  Y = quaternion[2]
  Z = quaternion[3]
  W = quaternion[0]

  xx = X*X
  xy = X*Y
  xz = X*Z
  xw = X*W

  yy = Y*Y
  yz = Y*Z
  yw = Y*W

  zz = Z*Z
  zw = Z*W

  output[0, 0] = 1.0-2.0*(yy+zz)
  output[0, 1] = 2.0*(xy-zw)
  output[0, 2] = 2.0*(xz+yw)

  output[1, 0] = 2.0*(xy+zw)
  output[1, 1] = 1.0-2.0*(xx+zz)
  output[1, 2] = 2.0*(yz-xw)

  output[2, 0] = 2.0*(xz-yw)
  output[2, 1] = 2.0*(yz+xw)
  output[2, 2] = 1.0-2.0*(xx+yy)

  return output


def rot2ea(R, output=None):
  if output is None:
    output = np.empty(3, dtype=np.float64)

  sy = np.linalg.norm(R[0:2, 0])
  singular = sy < 1e-6

  if not singular:
    x = atan2(R[2, 1], R[2, 2])
    y = atan2(-R[2, 0], sy)
    z = atan2(R[1, 0], R[0, 0])
  else:
    output[0:3] = np.nan
    return output
    #x = atan2(-R[1, 2], R[1, 1])
    #y = atan2(-R[2, 0], sy)
    #z = 0

  pi = np.pi
  if x > pi:
    x = x-2*pi
  if y > pi:
    y = y-2*pi
  if z > pi:
    z = z-2*pi

  output[0] = x
  output[1] = y
  output[2] = z
  return output


def get_grav_comp_efforts(robot, positions, gravity):
  g_norm = np.linalg.norm(gravity)
  if g_norm > 0.0:
    gravity = gravity/g_norm*9.81

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

