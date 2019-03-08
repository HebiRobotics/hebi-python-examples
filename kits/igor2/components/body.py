import hebi
import numpy as np
from numpy import matlib

from util.type_utils import assert_type
from util import math_utils


class BaseBody(object):
  """
  Base class for all body components of Igor
  """

  def __init__(self, val_lock, mass=0.0, com=[0.0, 0.0, 0.0]):
    self.__val_lock = val_lock
    self._mass = mass
    self._com = np.matrix(np.zeros((3, 1), dtype=np.float64))
    self._com[0, 0] = com[0]
    self._com[1, 0] = com[1]
    self._com[2, 0] = com[2]

  def _set_mass(self, mass):
    """
    Called by subclasses to update mass
    """
    self._mass = mass

  def _set_com(self, com):
    """
    Called by subclasses to update the center of mass
    """
    self._com[0:3, 0] = com

  def acquire_value_lock(self):
    """
    Used to acquire mutex for the parameters of this body component
    """
    self.__val_lock.acquire()

  def release_value_lock(self):
    """
    Used to release mutex for the parameters of this body component
    """
    self.__val_lock.release()

  @property
  def mass(self):
    """
    :return: The mass (in kilograms) of this body component
    :rtype:  float
    """
    return self._mass

  @property
  def com(self):
    """
    :return: The center of mass (in meters) of this body component in 3D Euclidean space
    :rtype:  np.matrix
    """
    return self._com


class PeripheralBody(BaseBody):
  """
  Base class for all peripheral body components of Igor (e.g., legs and arms)
  """

  def __init__(self, val_lock, group_indices, name, mass=None, com=None):
    if mass is not None and com is not None:
      super(PeripheralBody, self).__init__(val_lock, mass, com)
    else:
      super(PeripheralBody, self).__init__(val_lock)

    self._kin = hebi.robot_model.RobotModel()
    self._group_indices = group_indices
    self._name = name

    num_modules = len(group_indices)
    # ----------------------------------
    # Everything here is a column vector
    self._fbk_position = np.asmatrix(np.zeros((num_modules, 1), dtype=np.float64))
    self._fbk_position_cmd = np.asmatrix(np.zeros((num_modules, 1), dtype=np.float64))
    self._fbk_velocity = np.asmatrix(np.zeros((num_modules, 1), dtype=np.float32))
    self._fbk_velocity_err = np.asmatrix(np.zeros((num_modules, 1), dtype=np.float32))
    self._xyz_error = np.asmatrix(np.zeros((3, 1), dtype=np.float64))
    self._pos_error = np.asmatrix(np.zeros((6, 1), dtype=np.float64))
    self._vel_error = np.asmatrix(np.zeros((6, 1), dtype=np.float32))
    self._impedance_err = np.asmatrix(np.zeros((6, 1), dtype=np.float64))
    self._impedance_torque = np.asmatrix(np.zeros((num_modules, 1), dtype=np.float64))
    self._home_angles = np.asmatrix(np.zeros((num_modules, 1), dtype=np.float64))
    self._masses = None

    # Subclass populates these two empty list fields
    self._current_coms = list()
    self._current_fk = list()
    self._current_tip_fk = np.asmatrix(np.zeros((4, 4), dtype=np.float64))
    self._current_j_actual = np.asmatrix(np.zeros((6, num_modules), dtype=np.float64))
    self._current_j_actual_f = np.asmatrix(np.zeros((6, num_modules), dtype=np.float32))
    self._current_j_expected = np.asmatrix(np.zeros((6, num_modules), dtype=np.float64))
    self._current_j_expected_f = np.asmatrix(np.zeros((6, num_modules), dtype=np.float32))
    # Subclass populates this as size of (3, numOfCoMFrames)
    self._current_xyz = None

    # Because a `numpy.matrix` and `numpy.ndarray` are different types,
    # calling `A1` on a `numpy.matrix` incurs a surprising amount of overhead.
    # Due to this, we cache the `ndarray` views into certain numpy matrices.
    self._home_angles__flat = self._home_angles.A1
    self._fbk_position__flat = self._fbk_position.A1
    self._fbk_position_cmd__flat = self._fbk_position_cmd.A1
    self._fbk_velocity__flat = self._fbk_velocity.A1
    self._fbk_velocity_err__flat = self._fbk_velocity_err.A1

  def _set_masses(self, masses):
    length = len(masses)
    self._masses = np.asmatrix(np.empty((length, 1), dtype=np.float64))
    masses__flat = self._masses.A1
    np.copyto(masses__flat, masses)

  @property
  def _robot(self):
    return self._kin

  @property
  def name(self):
    """
    :return: The human readable name of this body component
    :rtype:  str
    """
    return self._name

  @property
  def home_angles(self):
    """
    :return: The home angle (in radians) positions of this body component
    :rtype:  np.array
    """
    return self._home_angles

  @home_angles.setter
  def home_angles(self, value):
    np.copyto(self._home_angles__flat, value)

  @property
  def group_indices(self):
    """
    :return: a list of integers corresponding to the modules
    this body represents in the Igor group
    :rtype:  list
    """
    return self._group_indices

  @property
  def current_coms(self):
    """
    :return:
    :rtype:
    """
    return self._current_coms

  @property
  def current_fk(self):
    """
    :return:
    :rtype:
    """
    return self._current_fk

  @property
  def current_tip_fk(self):
    """
    :return:
    :rtype:
    """
    return self._current_tip_fk

  @property
  def current_j_actual(self):
    """
    :return:
    :rtype:
    """
    return self._current_j_actual

  @property
  def current_j_expected(self):
    """
    :return:
    :rtype:
    """
    return self._current_j_expected

  def on_feedback_received(self, position, position_command, velocity, velocity_error):
    """
    Called by the Igor class when new feedback has been received.
    """
    indices = self._group_indices
    np.copyto(self._fbk_position__flat, position[indices])
    np.copyto(self._fbk_position_cmd__flat, position_command[indices])
    np.copyto(self._fbk_velocity__flat, velocity[indices])
    np.copyto(self._fbk_velocity_err__flat, velocity_error[indices])

  def get_grav_comp_efforts(self, positions, gravity):
    """
    :param positions: 
    :type positions:  np.array
    :param gravity:   
    :type gravity:    np.array
    
    :return: 
    :rtype:  np.array
    """
    kin = self._kin
    positions = positions[self._group_indices]
    return math_utils.get_grav_comp_efforts(kin, positions, gravity)

  def create_home_trajectory(self, positions, duration=3.0):
    """
    Create a trajectory from the current pose to the home pose.
    This is used on soft startup

    :param positions: 
    :type positions:  np.array
    :param duration:  The total time of the calculated trajectory (in seconds)
    :type duration:   float

    :rtype: hebi._internal.trajectory.Trajectory

    :raises ValueError: If ``duration`` is less than 1.0
    :raises TypeError:  If ``duration`` is not of type float
    """
    assert_type(duration, float, 'duration')
    if duration < 1.0:
      raise ValueError('duration must be greater than 1.0 second')
    indices = self._group_indices
    num_joints = len(indices)
    num_waypoints = 2
    dim = (num_joints, num_waypoints)

    current_positions = positions[indices]
    home_angles = self._home_angles

    times = np.array([0.0, duration], dtype=np.float64)
    pos = np.empty(dim, dtype=np.float64)
    vel = np.zeros(dim, dtype=np.float64)
    accel = vel

    pos[:, 0] = current_positions
    pos[:, 1] = home_angles.A1

    return hebi.trajectory.create_trajectory(times, pos, vel, accel)

  def update_position(self):
    """
    Update kinematics from feedback
    """
    positions = self._fbk_position__flat
    commanded_positions = self._fbk_position_cmd__flat

    robot = self._robot
    robot.get_forward_kinematics('com', positions, output=self._current_coms)
    robot.get_forward_kinematics('output', positions, output=self._current_fk)
    np.copyto(self._current_tip_fk, self._current_fk[-1])
    robot.get_jacobian_end_effector(positions, output=self._current_j_actual)
    robot.get_jacobian_end_effector(commanded_positions, output=self._current_j_expected)

    np.copyto(self._current_j_actual_f, self._current_j_actual)
    np.copyto(self._current_j_expected_f, self._current_j_expected)

    for i, entry in enumerate(self._current_coms):
      self._current_xyz[0:3, i] = entry[0:3, 3]

    masses = self._masses
    np.sum(np.multiply(self._current_xyz, matlib.repmat(masses.T, 3, 1)), axis=1, out=self._com)
    self._com *= 1.0/self.mass

  def reset_state(self):
    """
    Used when transitioning Igor back into idle mode.
    """
    self._fbk_position.fill(0)
    self._fbk_position_cmd.fill(0)
    self._fbk_velocity.fill(0)
    self._fbk_velocity_err.fill(0)
    self._xyz_error.fill(0)
    self._pos_error.fill(0)
    self._vel_error.fill(0)
    self._impedance_err.fill(0)
    self._impedance_torque.fill(0)
    self._current_tip_fk.fill(0)
    self._current_j_actual.fill(0)
    self._current_j_actual_f.fill(0)
    self._current_j_expected.fill(0)
    self._current_j_expected_f.fill(0)

    if self._current_xyz is not None:
      self._current_xyz.fill(0)

    # TODO: is this necessary?
    self._home_angles__flat = self._home_angles.A1
    self._fbk_position__flat = self._fbk_position.A1
    self._fbk_position_cmd__flat = self._fbk_position_cmd.A1
    self._fbk_velocity__flat = self._fbk_velocity.A1
    self._fbk_velocity_err__flat = self._fbk_velocity_err.A1
