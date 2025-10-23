import numpy as np
import hebi

from .body import BaseBody
from util import math_utils

from time import time


class Chassis(BaseBody):
  """
  Class representing the chassis of Igor
  """

  Velocity_P = 20.0
  Velocity_I = 3.0
  Velocity_D = 0.5

  def _create_trajectory(self):
    return hebi.trajectory.create_trajectory(self._traj_times,
                                             self._velocities,
                                             self._accels,
                                             self._jerks)

  def __init__(self, val_lock):
    super(Chassis, self).__init__(val_lock, mass=6.0, com=[0.00, 0.0, 0.10+0.3])
    self._user_commanded_directional_velocity = 0.0
    self._user_commanded_yaw_velocity = 0.0
    self._min_ramp_time = 0.5
    self._hip_pitch = 0.0
    self._hip_pitch_velocity = 0.0

    self._velocity_feedforward = 0.0
    self._velocity_error = 0.0
    self._velocity_error_cumulative = 0.0
    self._lean_angle_error = 0.0
    self._lean_angle_error_cumulative = 0.0
    self._cmd_chassis_vel_last = 0.0
    self._fbk_chassis_vel_last = 0.0

    self._i_term_adjust = -1.0

    self._calculated_lean_angle = 0.0

    # ---------------
    # Trajectory data
    num_cmds = 6

    self._traj_times = np.array([0.0, self._min_ramp_time], dtype=np.float64)

    # Column[0]: Trajectory calculated velocities at given point in time
    # Column[1]: User commanded velocities at given point in time
    self._velocities = np.zeros((num_cmds, 2), dtype=np.float64)

    # Column[0]: Trajectory calculated accelerations at given point in time
    # Column[1]: Always 0
    self._accels = np.zeros((num_cmds, 2), dtype=np.float64)

    # Column[0]: Trajectory calculated jerks at given point in time
    # Column[1]: Always 0
    self._jerks = np.zeros((num_cmds, 2), dtype=np.float64)

    self._trajectory = self._create_trajectory()
    self._time_s = None

  def update_time(self):
    """
    TODO: Document
    """
    self._time_s = time()

  def update_trajectory(self, user_commanded_knee_velocity, user_commanded_grip_velocity):
    """
    :param user_commanded_knee_velocity: The knee velocity calculated from joystick or user input
    :type user_commanded_knee_velocity:  float

    :param user_commanded_grip_velocity: The grip velocity (x,y,z) calculated from joystick or user input
    :type user_commanded_grip_velocity:  np.array
    """
    time_now = time()
    t = time_now - self._time_s
    self._time_s = time_now

    if t > self._trajectory.end_time:
      # Clamp to end time to prevent undesired trajectory calculations.
      # This can lead to issues when re-entering running mode after entering idle mode from running mode
      t = self._trajectory.end_time

    # ---------------------------------------------
    # Smooth the trajectories for various commands.
    # This will be the starting waypoint for the new trajectory.
    # The end waypoint will be the desired (user commanded) values.
    vel_now, acc_now, jerk_now = self._trajectory.get_state(t)

    # Start waypoint velocities
    self._velocities[0:6, 0] = vel_now

    # End waypoint velocities
    self._velocities[0, 1] = self._user_commanded_directional_velocity
    self._velocities[1, 1] = self._user_commanded_yaw_velocity
    self._velocities[2, 1] = user_commanded_knee_velocity
    self._velocities[3:6, 1] = user_commanded_grip_velocity

    # Start waypoint accel
    self._accels[0:6, 0] = acc_now

    # Start waypoint jerk
    self._jerks[0:6, 0] = jerk_now

    # End waypoint accel and jerk are always zero
    self._trajectory = self._create_trajectory()

  def integrate_step(self, dt):
    """
    Called by Igor. User should not call this directly.

    :param dt: timestep used in integrating
    :type dt:  float
    """
    self._hip_pitch = self._hip_pitch+self._hip_pitch_velocity*dt

  def update_velocity_controller(self, dt, velocities, wheel_radius,
                                 height_com, fbk_lean_angle_vel,
                                 robot_mass, fbk_lean_angle):
    """
    :param dt:
    :param velocities:
    :param wheel_radius:
    :param height_com:
    :param fbk_lean_angle_vel:
    :param robot_mass:
    :param fbk_lean_angle:
    """
    velP = Chassis.Velocity_P
    # For calibrating Igor (finding the ideal I term for the individual kit), uncomment this out and find a good value from the `a5` slider (Mobile IO only)
    # BE CAREFUL! adjusting the I term too quickly can cause the balance controller to command a very fast rotation to the wheels,
    # possibly causing Igor to fall forwards/backwards without anybody to support the kit.
    velI = Chassis.Velocity_I * (self._i_term_adjust + 1.0)
    #velI = Chassis.Velocity_I
    velD = Chassis.Velocity_D

    inv_dt = 1.0/dt
    l_wheel_vel = math_utils.zero_on_nan(velocities[0])
    r_wheel_vel = math_utils.zero_on_nan(velocities[1])

    fbk_chassis_vel = wheel_radius*(l_wheel_vel-r_wheel_vel)*0.5+(height_com*fbk_lean_angle_vel)
    cmd_chassis_vel = self._velocities[0, 0]
    chassis_vel_error = cmd_chassis_vel-fbk_chassis_vel
    chassis_vel_error_cumulative = self._velocity_error_cumulative+(chassis_vel_error*dt)
    chassis_vel_error_cumulative = np.clip(chassis_vel_error_cumulative, -50.0, 50.0)

    cmd_chassis_accel = (cmd_chassis_vel-self._cmd_chassis_vel_last)*inv_dt
    self._cmd_chassis_vel_last = cmd_chassis_vel
    chassis_accel = (fbk_chassis_vel-self._fbk_chassis_vel_last)*inv_dt
    self._fbk_chassis_vel_last = fbk_chassis_vel
    lean_feedforward = 0.1*robot_mass*cmd_chassis_accel/height_com
    velocity_feedforward = cmd_chassis_vel/wheel_radius

    cmd_lean_angle = (velP * chassis_vel_error) + (velI * chassis_vel_error_cumulative) + (
          velD * chassis_accel) + lean_feedforward

    # Store so we can save to logs
    self._cmd_lean_angle = cmd_lean_angle
    self._lean_ff = lean_feedforward

    lean_angle_error = fbk_lean_angle-cmd_lean_angle
    lean_angle_error_cumulative = self._lean_angle_error_cumulative+(lean_angle_error*dt)
    lean_angle_error_cumulative = np.clip(lean_angle_error_cumulative, -0.2, 0.2)

    self._velocity_feedforward = velocity_feedforward
    self._lean_feedforward = lean_feedforward
    self._velocity_error = chassis_vel_error
    self._velocity_error_cumulative = chassis_vel_error_cumulative
    self._lean_angle_error = lean_angle_error
    self._lean_angle_error_cumulative = lean_angle_error_cumulative

  @property
  def velocity_feedforward(self):
    """
    :return: The current velocity feedforward term
    :rtype:  float
    """
    return self._velocity_feedforward

  @property
  def lean_feedforward(self):
    """
    :return: The current lean feedforward term
    :rtype:  float
    """
    return self._lean_feedforward

# ------------------------------------------------------------------------------
# Calculated Errors
# ------------------------------------------------------------------------------

  @property
  def velocity_error(self):
    """
    :return: The current velocity error of the velocity controller
    :rtype:  float
    """
    return self._velocity_error

  @property
  def velocity_error_cumulative(self):
    """
    :return: The current cumulative velocity error of the velocity controller
    :rtype:  float
    """
    return self._velocity_error_cumulative

  @property
  def lean_angle_error(self):
    """
    :return: The current lean angle error of the velocity controller
    :rtype:  float
    """
    return self._lean_angle_error

  @property
  def lean_angle_error_cumulative(self):
    """
    :return: The current cumulative lean angle error of the velocity controller
    :rtype:  float
    """
    return self._lean_angle_error_cumulative

# ------------------------------------------------------------------------------
# User Commanded Properties
# ------------------------------------------------------------------------------

  @property
  def user_commanded_directional_velocity(self):
    """
    :rtype:  float
    """
    return self._velocities[0, 1]

  @property
  def user_commanded_yaw_velocity(self):
    """
    :rtype:  float
    """
    return self._velocities[1, 1]

  @property
  def user_commanded_knee_velocity(self):
    """
    :rtype:  float
    """
    return self._velocities[2, 1]

# ------------------------------------------------------------------------------
# Calculated Properties
# ------------------------------------------------------------------------------

  @property
  def calculated_directional_velocity(self):
    """
    :return: the calculated directional velocity
    :float:  float
    """
    return self._velocities[0, 0]

  @property
  def calculated_yaw_velocity(self):
    """
    :return: the calculated yaw velocity
    :float:  float
    """
    return self._velocities[1, 0]

  @property
  def calculated_knee_velocity(self):
    """
    :return: the calculated knee velocity (in rad/s)
    :float:  float
    """
    return self._velocities[2, 0]

  @property
  def calculated_grip_velocity(self):
    """
    :return:
    :float:  np.array
    """
    return self._velocities[3:6, 0]

  @property
  def i_term_adjust(self):
    return self._i_term_adjust

# ------------------------------------------------------------------------------
# User Commanded mutators
# ------------------------------------------------------------------------------

  def set_directional_velocity(self, velocity):
    """
    :param velocity:
    :return:
    """
    self.acquire_value_lock()
    self._user_commanded_directional_velocity = velocity
    self.release_value_lock()

  def set_yaw_velocity(self, velocity):
    """
    :param velocity:
    :return:
    """
    self.acquire_value_lock()
    self._user_commanded_yaw_velocity = velocity
    self.release_value_lock()

  def set_i_term_adjustment(self, value):
    self.acquire_value_lock()
    self._i_term_adjust = value
    self.release_value_lock()

  def reset_state(self):
    self._user_commanded_directional_velocity = 0.0
    self._user_commanded_yaw_velocity = 0.0
    self._hip_pitch = 0.0
    self._hip_pitch_velocity = 0.0
    self._velocity_feedforward = 0.0
    self._velocity_error = 0.0
    self._velocity_error_cumulative = 0.0
    self._lean_angle_error = 0.0
    self._lean_angle_error_cumulative = 0.0
    self._cmd_chassis_vel_last = 0.0
    self._fbk_chassis_vel_last = 0.0
    self._calculated_lean_angle = 0.0
    self._i_term_adjust = -1.0
