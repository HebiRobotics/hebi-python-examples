import numpy as np

from .body import PeripheralBody
from util import math_utils


class Leg(PeripheralBody):
  """
  Represents a leg (and wheel)
  """

  damper_gains = np.matrix([2.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  spring_gains = np.matrix([400.0, 0.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  roll_gains = np.matrix([0.0, 0.0, 10.0, 0.0, 0.0, 0.0], dtype=np.float64).T

  def __init__(self, val_lock, name, group_indices):
    assert name == 'Left' or name == 'Right'
    super(Leg, self).__init__(val_lock, group_indices, name)

    self._name = name
    base_frame = np.identity(4, dtype=np.float64)

    hip_t = np.identity(4, dtype=np.float64)
    hip_t[0:3, 0:3] = math_utils.rotate_x(np.pi*0.5)
    hip_t[0:3, 3] = [0.0, 0.0225, 0.055]
    self._hip_t = hip_t

    kin = self._robot
    kin.add_actuator('X5-9')
    kin.add_link('X5', extension=0.375, twist=np.pi)
    kin.add_actuator('X5-4')
    kin.add_link('X5', extension=0.325, twist=np.pi)

    home_knee_angle = np.deg2rad(130)
    home_hip_angle = (np.pi+home_knee_angle)*0.5

    if name == 'Left':
      self._direction = 1.0
      home_angles = np.array([home_hip_angle, home_knee_angle], dtype=np.float64)
      base_frame[0:3, 3] = [0.0, 0.15, 0.0]
      base_frame[0:3, 0:3] = math_utils.rotate_x(np.pi*-0.5)
    else:
      self._direction = -1.0
      home_angles = np.array([-home_hip_angle, -home_knee_angle], dtype=np.float64)
      base_frame[0:3, 3] = [0.0, -0.15, 0.0]
      base_frame[0:3, 0:3] = math_utils.rotate_x(np.pi*0.5)

    kin.base_frame = base_frame
    self.home_angles = home_angles
    masses = kin.masses
    self._set_mass(np.sum(masses))
    self._set_masses(masses)

    # ----------------------
    # Populate cached fields
    output_frame_count = kin.get_frame_count('output')
    com_frame_count = kin.get_frame_count('CoM')

    self._current_xyz = np.asmatrix(np.zeros((3, com_frame_count), dtype=np.float64))

    for i in range(output_frame_count):
      self._current_fk.append(np.asmatrix(np.zeros((4, 4), dtype=np.float64)))

    for i in range(com_frame_count):
      self._current_coms.append(np.asmatrix(np.zeros((4, 4), dtype=np.float64)))

    # Calculate home FK
    self._hip_angle = home_hip_angle
    self._knee_angle = home_knee_angle
    self._knee_velocity = 0.0
    self._user_commanded_knee_velocity = 0.0
    self._knee_angle_max = 2.65
    self._knee_angle_min = 0.65
    self._e_term = np.asmatrix(np.empty((6, 1), dtype=np.float64))

    # Additionally calculate commanded endeffector position
    self._current_cmd_tip_fk = np.asmatrix(np.zeros((4, 4), dtype=np.float64))

  def integrate_step(self, dt, knee_velocity):
    """
    Called by Igor. User should not call this directly.

    :param dt:            integral timestep
    :param knee_velocity: The calculated knee velocity
                          at the given instance in time
    """
    if (self._knee_angle > self._knee_angle_max) and (knee_velocity > 0.0) or\
       (self._knee_angle < self._knee_angle_min) and (knee_velocity < 0.0):
      # software controlled joint limit for the knee
      knee_velocity = 0.0

    self._knee_velocity = knee_velocity
    self._knee_angle = self._knee_angle+knee_velocity*dt
    self._hip_angle = (np.pi+self._knee_angle)*0.5

  def update_position(self):
    """
    Updates calculations based on the position of the leg
    at the given point in time
    """
    super(Leg, self).update_position()
    self._robot.get_forward_kinematics('endeffector', self._fbk_position_cmd, output=[self._current_cmd_tip_fk])

  def update_command(self, group_command, roll_angle, soft_start):
    """
    Write into the command object based on the current state of the leg

    :param group_command:
    :param roll_angle:
    :param soft_start:
    :return:
    """
    indices = self.group_indices
    hip_idx = indices[0]
    knee_idx = indices[1]

    hip_cmd = group_command[hip_idx]
    knee_cmd = group_command[knee_idx]

    # -------------------------------
    # Calculate position and velocity
    knee_velocity = self._direction*self._knee_velocity
    hip_cmd.position = self._direction*self._hip_angle
    hip_cmd.velocity = knee_velocity*0.5
    knee_cmd.position = self._direction*self._knee_angle
    knee_cmd.velocity = knee_velocity

    # ----------------
    # Calculate effort

    # Calculate the current positional error 
    np.subtract(self._current_cmd_tip_fk[0:3, 3], self.current_tip_fk[0:3, 3], out=self._xyz_error)
    np.copyto(self._pos_error[0:3], self._xyz_error)
    # Calculate the current velocity error by:
    #  multiplying the current jacobian at the endeffector frame
    # by the:
    #  difference of the commanded velocity feedfback and actual velocity feedfback
    np.dot(self._current_j_actual_f, self._fbk_velocity_err, out=self._vel_error)
    roll_sign = self._direction

    # piecewise multiply the error terms by the predefined gains
    np.multiply(Leg.spring_gains, self._pos_error, out=self._pos_error)
    np.multiply(Leg.damper_gains, self._vel_error, out=self._vel_error)
    # `self._e_term` is an intermediate value used to calculate the impedance error
    np.multiply(Leg.roll_gains, roll_sign*roll_angle, out=self._e_term)

    # add the 3 error terms (pos+vel+e) to find the impedance error
    np.add(self._pos_error, self._vel_error, out=self._impedance_err)
    np.add(self._impedance_err, self._e_term, out=self._impedance_err)
    # Multiply the jacobian matrix at the endeffector 
    # by the impedance error to find the impedance torque, and scale it
    # by the soft startup scale
    np.dot(self.current_j_actual.T, self._impedance_err, out=self._impedance_torque)
    np.multiply(self._impedance_torque, soft_start, out=self._impedance_torque)

    hip_cmd.effort = self._impedance_torque[0, 0]
    knee_cmd.effort = self._impedance_torque[1, 0]

  @property
  def hip_angle(self):
    """
    :return: The current hip position (in radians)
    :rtype:  float
    """
    return self._hip_angle

  @property
  def knee_angle(self):
    """
    :return: The current knee angle (in radians)
    :rtype:  float
    """
    return self._knee_angle

  @property
  def user_commanded_knee_velocity(self):
    """
    :return: (in rad/s)
    :rtype:  float
    """
    return self._user_commanded_knee_velocity

  def set_knee_velocity(self, vel):
    """
    :param vel: (in rad/s)
    :type vel:  float
    """
    self.acquire_value_lock()
    self._user_commanded_knee_velocity = vel
    self.release_value_lock()
