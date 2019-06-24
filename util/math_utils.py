import numpy as np
from math import atan2, cos, isnan, sin


def sign(val):
  """
  Mimics MATLAB's sign function

  :param val: Input value
  :type val:  float
  :return:    The sign of ``val``
  :rtype:     float
  """
  if val > 0.0:
    return 1.0
  elif val == 0.0:
    return 0.0
  else:
    return -1.0


def zero_on_nan(val):
  """
  :param val: Input value
  :type val:  float
  :return: 0.0 if ``val`` is nan, otherwise ``val``
  :rtype:  float
  """
  if isnan(val):
    return 0.0
  else:
    return val


def any_nan(mat):
  """
  :param mat:
  :return: 
  :rtype:  bool
  """
  return np.any(np.isnan(mat))


def assert_not_nan(val, msg):
  """
  :param msg:
  :type msg:  str
  """
  if isnan(val):
    raise ValueError('{0} is nan'.format(msg))


def rotate_x(angle, dtype=np.float64, output=None):
  """
  Apply an X rotation to the matrix

  :param angle:
  :param dtype:

  :param output:
  :type output:  np.ndarray, NoneType

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
  Apply a Y rotation to the matrix

  :param angle:
  :param dtype:

  :param output:
  :type output:  np.ndarray, NoneType

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
  Apply a Z rotation to the matrix

  :param angle:
  :param dtype:

  :param output:
  :type output:  np.ndarray, NoneType

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
  """
  Retrieve the rotation matrix for the provided rotation quaternion

  :param quaternion:
  :param output:
  :type output:  np.ndarray, NoneType
  :return:
  """

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


def gravity_from_quaternion(quaternion, output=None):
  """
  Retrieve the gravity vector from the given quaternion
  """
  if output is None:
    output = np.empty(3, dtype=np.float32)

  X = quaternion[1]
  Y = quaternion[2]
  Z = quaternion[3]
  W = quaternion[0]

  xx = X*X
  xz = X*Z
  xw = X*W
  yy = Y*Y
  yz = Y*Z
  yw = Y*W

  output[0] = -2.0*(xz-yw)
  output[1] = -2.0*(yz+xw)
  output[2] = -1.0+2.0*(xx+yy)

  return output


def rot2ea(R, output=None):
  """
  Retrieve the Euler angle from the input rotation matrix

  :param R:
  :param output:
  :type output:  np.ndarray, NoneType

  :return: 3 element array representing the Euler rotation
  :rtype:  np.ndarray
  """
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

  pi = np.pi
  n_pi2 = pi*-2.0
  if x > pi:
    x = x+n_pi2
  if y > pi:
    y = y+n_pi2
  if z > pi:
    z = z+n_pi2

  output[0] = x
  output[1] = y
  output[2] = z
  return output


def get_grav_comp_efforts(robot, positions, gravity, output=None):
  """
  :param robot:
  :param positions:
  :param gravity:

  :param output:
  :type output:  np.ndarray, NoneType

  :return:
  :rtype:  np.ndarray
  """
  g_norm = np.linalg.norm(gravity)
  if g_norm > 0.0:
    gravity = gravity/g_norm*9.81

  jacobians = robot.get_jacobians('CoM', positions)
  if output is None:
    comp_torque = np.zeros((robot.dof_count, 1))
  else:
    comp_torque = output
  wrench = np.zeros(6)
  num_frames = robot.get_frame_count('CoM')

  masses = robot.masses

  for i in range(num_frames):
    # Add the torques for each joint to support the mass at this frame
    wrench[0:3] = gravity*masses[i]
    comp_torque += jacobians[i].T*np.reshape(wrench, (6, 1))

  ret = np.squeeze(comp_torque)
  return ret


def get_dynamic_comp_efforts(fbk_positions, cmd_positions, cmd_velocities, cmd_accels, robot, dt=1e-3):
  """
  :param fbk_positions:
  :param cmd_positions:
  :param cmd_velocities:
  :param cmd_accels:
  :param robot:
  :param dt:

  :return:
  :rtype:  np.ndarray
  """
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

  for module in range(len(masses)):
    # Calculate XYZ accelerations of the CoM
    lastv = cmd_frames_last[module]
    nowv = cmd_frames_now[module]
    nextv = cmd_frames_next[module]

    accel = ((lastv+nextv)-(2*nowv))*dt_s_inv

    # Set translational part of wrench vector (rotational stays zero)
    wrench[0:3] = np.reshape(accel*masses[module], (3,))

    # compEffort = J' * wrench
    efforts += jacobians[module].T*np.reshape(wrench, (6, 1))

  return np.squeeze(efforts)

