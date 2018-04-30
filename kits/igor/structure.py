import math, sys, threading
import numpy as np
import numpy.matlib as matlib

from . import DemoUtils, Joystick
from .config import Igor2Config
from .event_handlers import register_igor_event_handlers

# kits.util._internal.type_utils
from ..util._internal.type_utils import assert_instance, assert_type
from ..util import math_func
import hebi

from time import sleep, time
from functools import partial as funpart


# ------------------------------------------------------------------------------
# Base Classes
# ------------------------------------------------------------------------------


class BaseBody(object):

  def __init__(self, val_lock, mass=0.0, com=[0.0, 0.0, 0.0]):
    self.__val_lock = val_lock
    self._mass = mass
    self._com = np.asarray(com, dtype=np.float64)

  def _set_mass(self, mass):
    self._mass = mass

  def _set_com(self, com):
    self._com = np.asarray(com, dtype=np.float64)

  def acquire_value_lock(self):
    self.__val_lock.acquire()

  def release_value_lock(self):
    self.__val_lock.release()

  @property
  def mass(self):
    return self._mass

  @property
  def com(self):
    return self._com


class PeripheralBody(BaseBody):

  def __init__(self, val_lock, group_indices, name, mass=None, com=None):
    if mass != None and com != None:
      super(PeripheralBody, self).__init__(val_lock, mass, com)
    else:
      super(PeripheralBody, self).__init__(val_lock)

    self._group_indices = group_indices
    self._name = name
    self._kin = hebi.robot_model.RobotModel()
    self._home_angles = None
    self._masses_t = None

    self._current_coms = None
    self._current_fk = None
    self._current_tip_fk = None
    self._current_j_actual = None
    self._current_j_expected = None
    self._current_xyz = None

  @property
  def _robot(self):
    return self._kin

  @property
  def name(self):
    return self._name

  @property
  def home_angles(self):
    return self._home_angles

  @home_angles.setter
  def home_angles(self, value):
    self._home_angles = value

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
    return self._current_coms

  @property
  def current_fk(self):
    return self._current_fk

  @property
  def current_tip_fk(self):
    return self._current_tip_fk

  @property
  def current_j_actual(self):
    return self._current_j_actual

  @property
  def current_j_expected(self):
    return self._current_j_expected

  def _set_masses_t(self, masses):
    self._masses_t = masses

  def get_grav_comp_efforts(self, positions, gravity):
    kin = self._kin
    positions = positions[self._group_indices]
    return math_func.get_grav_comp_efforts(kin, positions, gravity)

  def create_home_trajectory(self, positions, duration=3.0):
    """
    Create a trajectory from the current pose to the home pose.
    This is used on soft startup
    ^^^ Is this the correct usage of "pose" ???
    :return:
    """
    assert_type(duration, float, 'duration')
    if duration < 1.0:
      raise ValueError('duration must be greater than 1.0 second')
    num_joints = len(self._group_indices)
    num_waypoints = 2
    dim = (num_joints, num_waypoints)

    current_positions = positions[self._group_indices]
    home_angles = self._home_angles

    times = np.array([0.0, duration], dtype=np.float64)
    pos = np.empty(dim, dtype=np.float64)
    vel = np.zeros(dim, dtype=np.float64)
    accel = np.zeros(dim, dtype=np.float64)

    pos[:, 0] = current_positions
    pos[:, 1] = home_angles

    return hebi.trajectory.create_trajectory(times, pos, vel, accel)

  def update_position(self, positions, commanded_positions):
    """
    Upate kinematics from feedback
    TODO: document and CLEAN UP
    :param positions:
    :param commanded_positions:
    """
    positions = positions[self._group_indices]
    commanded_positions = commanded_positions[self._group_indices]

    robot = self._robot
    coms = robot.get_forward_kinematics('com', positions)
    fk = robot.get_forward_kinematics('output', positions)
    tip_fk = fk[-1]
    jacobian_actual = robot.get_jacobian_end_effector(positions)
    jacobian_expected = robot.get_jacobian_end_effector(commanded_positions)

    masses_t = self._masses_t
    xyz = np.squeeze([entry[0:3, 3] for entry in coms]).T
    com = np.sum(np.multiply(xyz, matlib.repmat(masses_t, 3, 1)), axis=1) / self.mass

    self._current_coms = coms
    self._current_fk = fk
    self._current_tip_fk = tip_fk
    self._current_j_actual = jacobian_actual
    self._current_j_expected = jacobian_expected
    self._current_xyz = xyz
    self._set_com(com)


# ------------------------------------------------------------------------------
# Kinematics Classes
# ------------------------------------------------------------------------------


class Chassis(BaseBody):

  Velocity_P = 15.0
  Velocity_I = 0.1
  Velocity_D = .3

  def _create_trajectory(self):
    return hebi.trajectory.create_trajectory(self._traj_times,
                                             self._velocities,
                                             self._accels,
                                             self._jerks)

  def __init__(self, val_lock):
    super(Chassis, self).__init__(val_lock, mass=6.0, com=[0.0, 0.0, 0.10+0.3])
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

    self._calculated_lean_angle = 0.0

    # ---------------
    # Trajectory data
    num_cmds = 6

    self._traj_times = np.array([0.0, self._min_ramp_time], dtype=np.float64)

    # Column[0]: `chassisVelNow` in MATLAB
    #   - Trajectory calculated velocities at given point in time
    # Column[1]: `chassisCmdVel` in MATLAB
    #   - User commanded velocities at given point in time
    self._velocities = np.zeros((num_cmds, 2), dtype=np.float64)

    # Column[0]: `chassisAccNow` in MATLAB
    #  - Trajectory calculated accelerations at given point in time
    # Column[1]: Always 0
    self._accels = np.zeros((num_cmds, 2), dtype=np.float64)

    # Column[0]: `chassisJerkNow` in MATLAB
    #  - Trajectory calculated jerks at given point in time
    # Column[1]: Always 0
    self._jerks = np.zeros((num_cmds, 2), dtype=np.float64)

    self._trajectory = self._create_trajectory()
    self._time_s = None

  def update_time(self):
    self._time_s = time()

  def update_trajectory(self, user_commanded_knee_velocity, user_commanded_grip_velocity):
    """
    TODO: Document

    :param user_commanded_knee_velocity:
    :param user_commanded_grip_velocity:
    """
    time_now = time()
    t = time_now - self._time_s
    self._time_s = time_now

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
    :param dt: integral timestep
    """
    self._hip_pitch = self._hip_pitch+self._hip_pitch_velocity*dt

  def update_velocity_controller(self, dt, velocities, wheel_radius,
                                 height_com, fbk_lean_angle_vel,
                                 robot_mass, fbk_lean_angle):
    velP = Chassis.Velocity_P
    velI = Chassis.Velocity_I
    velD = Chassis.Velocity_D

    inv_dt = 1.0/dt
    l_wheel_vel = math_func.zero_on_nan(velocities[0])
    r_wheel_vel = math_func.zero_on_nan(velocities[1])

    # fbkChassisVel = wheelRadius * mean(direction.*fbk.velocity(1:2)) + heightCoM*leanAngleVel
    fbk_chassis_vel = wheel_radius*(l_wheel_vel-r_wheel_vel)*0.5+(height_com*fbk_lean_angle_vel)
    # cmdChassisVel = cmdVel
    cmd_chassis_vel = self._velocities[0, 0]
    # chassisVelError = cmdChassisVel - fbkChassisVel
    chassis_vel_error = cmd_chassis_vel-fbk_chassis_vel
    # chassisVelErrorCum = chassisVelErrorCum + chassisVelError*dt
    chassis_vel_error_cumulative = self._velocity_error_cumulative+(chassis_vel_error*dt)
    # chassisVelErrorCum = min(abs(chassisVelErrorCum),5/velI) * sign(chassisVelErrorCum)
    chassis_vel_error_cumulative = np.clip(chassis_vel_error_cumulative, -50.0, 50.0)

    # cmdChassisAccel = (cmdChassisVel - cmdChassisVelLast) / dt
    cmd_chassis_accel = (cmd_chassis_vel-self._cmd_chassis_vel_last)*inv_dt
    # cmdChassisVelLast = cmdChassisVel
    self._cmd_chassis_vel_last = cmd_chassis_vel
    # chassisAccel = (fbkChassisVel - fbkChassisVelLast) / dt
    chassis_accel = (fbk_chassis_vel-self._fbk_chassis_vel_last)*inv_dt
    # fbkChassisVelLast = fbkChassisVel
    self._fbk_chassis_vel_last = fbk_chassis_vel
    # leanFF = 0.1 * robotMass * cmdChassisAccel / heightCoM
    lean_feedforward = 0.1*robot_mass*cmd_chassis_accel/height_com
    # velFF = direction * cmdChassisVel / wheelRadius
    velocity_feedforward = cmd_chassis_vel/wheel_radius

    # cmdLeanAngle = velP * chassisVelError + velI * chassisVelErrorCum + velD * chassisAccel + leanFF
    cmd_lean_angle = (velP * chassis_vel_error) + (velI * chassis_vel_error_cumulative) + (
          velD * chassis_accel) + lean_feedforward

    # leanAngleError = fbkLeanAngle - cmdLeanAngle
    lean_angle_error = fbk_lean_angle-cmd_lean_angle
    # leanAngleErrorCum = leanAngleErrorCum + leanAngleError * dt
    lean_angle_error_cumulative = self._lean_angle_error_cumulative+(lean_angle_error*dt)
    # leanAngleErrorCum = min(abs(leanAngleErrorCum),.2) * sign(leanAngleErrorCum)
    lean_angle_error_cumulative = np.clip(lean_angle_error_cumulative, -0.2, 0.2)

    self._velocity_feedforward = velocity_feedforward
    self._lean_feedforward = lean_feedforward
    self._velocity_error = chassis_vel_error
    self._velocity_error_cumulative = chassis_vel_error_cumulative
    self._lean_angle_error = lean_angle_error
    self._lean_angle_error_cumulative = lean_angle_error_cumulative

    # TEMP
    self._calculated_lean_angle = cmd_lean_angle

  @property
  def velocity_feedforward(self):
    return self._velocity_feedforward

  @property
  def lean_feedforward(self):
    return self._lean_feedforward

# ------------------------------------------------------------------------------
# Calculated Errors
# ------------------------------------------------------------------------------

  @property
  def velocity_error(self):
    return self._velocity_error

  @property
  def velocity_error_cumulative(self):
    return self._velocity_error_cumulative

  @property
  def lean_angle_error(self):
    return self._lean_angle_error

  @property
  def lean_angle_error_cumulative(self):
    return self._lean_angle_error_cumulative

# ------------------------------------------------------------------------------
# User Commanded Properties
# ------------------------------------------------------------------------------

  @property
  def user_commanded_directional_velocity(self):
    return self._velocities[0, 1]

  @property
  def user_commanded_yaw_velocity(self):
    return self._velocities[1, 1]

  @property
  def user_commanded_knee_velocity(self):
    return self._velocities[2, 1]

# ------------------------------------------------------------------------------
# Calculated Properties
# ------------------------------------------------------------------------------

  @property
  def calculated_lean_angle(self):
    return self._calculated_lean_angle

  @property
  def calculated_directional_velocity(self):
    return self._velocities[0, 0]

  @property
  def calculated_yaw_velocity(self):
    return self._velocities[1, 0]

  @property
  def calculated_knee_velocity(self):
    return self._velocities[2, 0]

  @property
  def calculated_grip_velocity(self):
    return self._velocities[3:6, 0]

# ------------------------------------------------------------------------------
# User Commanded mutators
# ------------------------------------------------------------------------------

  def set_directional_velocity(self, velocity):
    """
    TODO: document
    :param velocity:
    :return:
    """
    self.acquire_value_lock()
    self._user_commanded_directional_velocity = velocity
    self.release_value_lock()

  def set_yaw_velocity(self, velocity):
    """
    TODO: document
    :param velocity:
    :return:
    """
    self.acquire_value_lock()
    self._user_commanded_yaw_velocity = velocity
    self.release_value_lock()


class Arm(PeripheralBody):

  damper_gains = np.matrix([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  spring_gains = np.matrix([100.0, 10.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64).T

  def __init__(self, val_lock, name, group_indices):
    assert name == 'Left' or name == 'Right'
    super(Arm, self).__init__(val_lock, group_indices, name)

    self._name = name
    self._user_commanded_grip_velocity = np.zeros(3, dtype=np.float64)
    base_frame = np.identity(4, dtype=np.float64)
    kin = self._robot
    kin.add_actuator('X5-4')

    if name == 'Left':
      self._direction = 1.0
      mounting = 'left-inside'
      base_frame[0:3, 3] = [0.0, 0.10, 0.20]
      home_angles = np.array([0.0, 20.0, 60.0, 0.0], dtype=np.float64)
    else:
      self._direction = -1.0
      mounting = 'right-inside'
      base_frame[0:3, 3] = [0.0, -0.10, 0.20]
      home_angles = np.array([0.0, -20.0, -60.0, 0.0], dtype=np.float64)

    kin.add_bracket('X5-HeavyBracket', mounting)
    kin.add_actuator('X5-9')
    kin.add_link('X5', extension=0.325, twist=0.0)
    kin.add_actuator('X5-4')
    kin.add_link('X5', extension=0.325, twist=np.pi)
    kin.add_actuator('X5-4')
    kin.base_frame = base_frame

    home_angles = np.deg2rad(home_angles)
    self.home_angles = home_angles
    self._home_fk = kin.get_forward_kinematics('output', home_angles)
    self._home_ef = self._home_fk[-1]
    self._set_mass(np.sum(kin.masses))
    self._set_masses_t(kin.masses.T)

    self._grip_pos = np.array(self._home_ef[0:3, 3])[:, 0]
    self._new_grip_pos = self._grip_pos.copy()
    self._joint_angles = home_angles.copy()
    self._joint_velocities = np.empty(4, np.float64)
    self._user_commanded_wrist_velocity = 0.0

    # Additionally, calculate determinant of jacobians
    self._current_det_actual = None
    self._current_det_expected = None

  def integrate_step(self, dt, positions, calculated_grip_velocity):
    """
    Called by Igor. User should not call this directly.
    :param dt:
    """

    # Make end effector velocities mirrored in Y
    adjusted_grip_velocity = calculated_grip_velocity.copy()
    adjusted_grip_velocity[1] = self._direction*adjusted_grip_velocity[1]

    self._new_grip_pos[:] = self._grip_pos+(adjusted_grip_velocity*dt)
    robot = self._robot
    positions = positions[self.group_indices]
    xyz_objective = hebi.robot_model.endeffector_position_objective(self._new_grip_pos)
    new_arm_joint_angs = robot.solve_inverse_kinematics(positions, xyz_objective)

    jacobian_new = robot.get_jacobian_end_effector(new_arm_joint_angs)[0:3, 0:3]
    det_J_new = abs(np.linalg.det(jacobian_new))

    if (self._current_det_expected < 0.010) and (det_J_new < self._current_det_expected):
      # Near singularity - don't command arm towards it
      self._joint_velocities[0:3] = 0.0
    else:
      try:
        self._joint_velocities[0:3] = np.linalg.solve(self.current_j_actual[0:3, 0:3], self._user_commanded_grip_velocity)
        self._joint_angles[0:3] = new_arm_joint_angs[0:3]
        self._grip_pos[:] = self._new_grip_pos
      except np.linalg.LinAlgError as lin:
        # This may happen still sometimes
        self._joint_velocities[0:3] = 0.0

    wrist_vel = self._direction*self._user_commanded_wrist_velocity
    self._joint_velocities[3] = self._joint_velocities[1]+self._joint_velocities[2]+wrist_vel
    self._joint_angles[3] = self._joint_angles[3]+(self._joint_velocities[3]*dt)

  @property
  def current_det_actual(self):
    return self._current_det_actual

  @property
  def current_det_expected(self):
    return self._current_det_expected

  @property
  def user_commanded_grip_velocity(self):
    return self._user_commanded_grip_velocity

  @property
  def user_commanded_wrist_velocity(self):
    return self._user_commanded_wrist_velocity

  @property
  def grip_position(self):
    return self._grip_pos

  def update_position(self, positions, commanded_positions):
    super(Arm, self).update_position(positions, commanded_positions)
    self._current_det_actual = np.linalg.det(self._current_j_actual[0:4, 0:4])
    self._current_det_expected = np.linalg.det(self._current_j_expected[0:4, 0:4])

  def update_command(self, group_command, positions, velocities, pose, soft_start):
    commanded_positions = self._joint_angles
    commanded_velocities = self._joint_velocities
    indices = self.group_indices

    positions = positions[indices]
    velocities = velocities[indices]

    # Positions and velocities are already known at this point - don't need to calculate them

    # ----------------
    # Calculate effort
    xyz_error = self._grip_pos-self.current_tip_fk[0:3, 3].ravel()
    pos_error = np.zeros((6, 1), dtype=np.float64)
    pos_error[0:3] = xyz_error.T
    vel_error = self.current_j_actual*np.matrix(commanded_velocities-velocities).T

    pos_dot = np.multiply(Arm.spring_gains, pos_error)
    vel_dot = np.multiply(Arm.damper_gains, vel_error)
    impedance_torque = self.current_j_actual.T*(pos_dot+vel_dot)
    grav_comp_torque = math_func.get_grav_comp_efforts(self._robot, positions, -pose[2, 0:3])

    effort = soft_start*impedance_torque.ravel()+grav_comp_torque.ravel()

    # Send commands
    idx = 0
    for i in self.group_indices:
      group_command[i].position = commanded_positions[idx]
      group_command[i].velocity = commanded_velocities[idx]
      group_command[i].effort = effort[0, idx]
      idx = idx + 1

  def set_x_velocity(self, value):
    self.acquire_value_lock()
    self._user_commanded_grip_velocity[0] = value
    self.release_value_lock()

  def set_y_velocity(self, value):
    self.acquire_value_lock()
    self._user_commanded_grip_velocity[1] = value
    self.release_value_lock()

  def set_z_velocity(self, value):
    self.acquire_value_lock()
    self._user_commanded_grip_velocity[2] = value
    self.release_value_lock()

  def set_wrist_velocity(self, value):
    self.acquire_value_lock()
    self._user_commanded_wrist_velocity = value
    self.release_value_lock()


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
    hip_t[0:3, 0:3] = math_func.rotate_x(np.pi*0.5)
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
      base_frame[0:3, 0:3] = math_func.rotate_x(np.pi*-0.5)
    else:
      self._direction = -1.0
      home_angles = np.array([-home_hip_angle, -home_knee_angle], dtype=np.float64)
      base_frame[0:3, 3] = [0.0, -0.15, 0.0]
      base_frame[0:3, 0:3] = math_func.rotate_x(np.pi*0.5)

    kin.base_frame = base_frame
    self.home_angles = home_angles
    masses = kin.masses
    self._set_mass(np.sum(masses))
    self._set_masses_t(masses.T)

    self._hip_angle = home_hip_angle
    self._knee_angle = home_knee_angle
    self._knee_velocity = 0.0
    self._user_commanded_knee_velocity = 0.0
    self._knee_angle_max = 2.65
    self._knee_angle_min = 0.65

    # Additionally calculate commanded end effector position
    self._current_cmd_tip_fk = None

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
    self._hip_angle = np.pi/2.0+self._knee_angle/2.0

  def update_position(self, positions, commanded_positions):
    super(Leg, self).update_position(positions, commanded_positions)
    self._current_cmd_tip_fk = self._robot.get_forward_kinematics('endeffector', commanded_positions[self.group_indices])[0]

  def update_command(self, group_command, vel_error, roll_angle, soft_start):
    indices = self.group_indices
    hip_idx = indices[0]
    knee_idx = indices[1]

    hip_cmd = group_command[hip_idx]
    knee_cmd = group_command[knee_idx]

    vel_error = np.matrix(vel_error[indices]).T

    # -------------------------------
    # Calculate position and velocity
    hip_cmd.position = self._direction*self._hip_angle
    hip_cmd.velocity = self._direction*self._knee_velocity*0.5
    knee_cmd.position = self._direction*self._knee_angle
    knee_cmd.velocity = self._direction*self._knee_velocity

    # ----------------
    # Calculate effort
    xyz_error = self._current_cmd_tip_fk[0:3, 3]-self.current_tip_fk[0:3, 3]
    pos_error = np.zeros((6, 1), dtype=np.float64)
    pos_error[0:3] = xyz_error
    vel_error = self.current_j_actual*vel_error
    roll_sign = self._direction

    pos_dot = np.multiply(Leg.spring_gains, pos_error)
    vel_dot = np.multiply(Leg.damper_gains, vel_error)
    e_term = np.multiply(Leg.roll_gains, roll_angle)*roll_sign
    impedance_torque = self.current_j_actual.T*(pos_dot+vel_dot+e_term)
    impedance_torque = impedance_torque.T*soft_start

    hip_cmd.effort = impedance_torque[0, 0]
    knee_cmd.effort = impedance_torque[0, 1]

  @property
  def hip_angle(self):
    return self._hip_angle

  @property
  def knee_angle(self):
    return self._knee_angle

  @property
  def user_commanded_knee_velocity(self):
    return self._user_commanded_knee_velocity

  def set_knee_velocity(self, vel):
    self.acquire_value_lock()
    self._user_commanded_knee_velocity = vel
    self.release_value_lock()


# ------------------------------------------------------------------------------
# Helper Functions for Igor class
# ------------------------------------------------------------------------------


def on_error_find_joystick(group, command):
  command.led.color = 'white'
  group.send_command(command)
  sleep(0.1)
  command.led.color = 'magenta'
  group.send_command(command)
  sleep(0.1)


def get_first_joystick():
  for i in range(Joystick.joystick_count()):
    try:
      return Joystick.at_index(i)
    except:
      pass


def set_command_subgroup_pve(group_command, pos, vel, effort, indices):
  """
  Set position, velocity, and effort for certain modules in a group

  :param group_command:
  :param pos:
  :param vel:
  :param effort:
  :param indices:
  :return:
  """
  idx = 0
  if effort is None:
    for i in indices:
      cmd = group_command[i]
      cmd.position = pos[idx]
      cmd.velocity = vel[idx]
      idx = idx + 1
  else:
    for i in indices:
      cmd = group_command[i]
      cmd.position = pos[idx]
      cmd.velocity = vel[idx]
      cmd.effort = effort[idx]
      idx = idx + 1


def create_group(config, has_camera):
  """
  Used by :class:`Igor` to create the group to interface with modules

  :param config: The runtime configuration
  :type config:  Igor2Config
  :param has_camera: 
  :type has_camera:  bool
  """
  imitation = config.is_imitation

  if imitation:
    if has_camera:
      num_modules = len(config.module_names)
    else:
      num_modules = len(config.module_names_no_cam)

    from hebi.util import create_imitation_group
    return create_imitation_group(num_modules)
  else:
    if has_camera:
      names = config.module_names
    else:
      names = config.module_names_no_cam
    families = [config.family]
    lookup = hebi.Lookup()

    def connect():
      group = lookup.get_group_from_names(families, names)
      if group == None:
        raise RuntimeError()
      elif group.size != len(names):
        raise RuntimeError()
      return group

    # Let the lookup object discover modules, before trying to connect
    sleep(2.0)
    return DemoUtils.retry_on_error(connect)


# Used for Igor background controller thread
if sys.version_info[0] == 3:
  is_main_thread_active = lambda: threading.main_thread().is_alive()
else:
  is_main_thread_active = lambda: any((i.name == "MainThread") and i.is_alive() for i in threading.enumerate())


# ------------------------------------------------------------------------------
# Debugging stuff
# ------------------------------------------------------------------------------

from . import demo_gui

# ------------------------------------------------------------------------------
# Igor Class
# ------------------------------------------------------------------------------


class Igor(object):

  Lean_P = 1.0
  Lean_I = 20.0
  Lean_D = 10.0

# ------------------------------------------------
# Temp Props
# ------------------------------------------------

# chassis.velocity_feedforward
# chassis.lean_angle_error
# chassis.lean_angle_error_cumulative
# chassis.calculated_directional_velocity
# chassis.calculated_yaw_velocity
# chassis.calculated_knee_velocity
# chassis.calculated_grip_velocity

  @property
  def roll_angle(self):
    return self._roll_angle

  @property
  def pitch_angle(self):
    return self._pitch_angle

  @property
  def height_com(self):
    return self._height_com

  @property
  def feedback_lean_angle(self):
    return self._feedback_lean_angle

  @property
  def feedback_lean_angle_velocity(self):
    return self._feedback_lean_angle_velocity

# ------------------------------------------------
# Helper functions
# ------------------------------------------------

  def _ensure_started(self):
    """
    Contract: This method assumes the caller has acquired `_state_lock`
    """
    if not self._started:
      self._state_lock.release()
      raise RuntimeError('Igor has not been started (igor.start() was not called)')

  def _pending_quit(self):
    """
    Contract: This method assumes the caller has acquired `_state_lock`
    """
    return self._quit_flag

  def _continue(self):
    """
    Contract: This method assumes the caller has acquired `_state_lock`
    """
    return is_main_thread_active() and not self._pending_quit()

# ------------------------------------------------
# Calculations
# ------------------------------------------------

  def _update_com(self, positions, commanded_positions):
    """
    TODO: Document
    :param positions:
    :param commanded_positions:
    """
    l_arm = self._left_arm
    r_arm = self._right_arm
    l_leg = self._left_leg
    r_leg = self._right_leg

    l_arm.update_position(positions, commanded_positions)
    r_arm.update_position(positions, commanded_positions)
    l_leg.update_position(positions, commanded_positions)
    r_leg.update_position(positions, commanded_positions)

    coms = self._coms
    coms[0:3, 0] = l_leg.com
    coms[0:3, 1] = r_leg.com
    coms[0:3, 2] = l_arm.com
    coms[0:3, 3] = r_arm.com

    masses = self._masses

    self._com = np.sum(np.multiply(coms, matlib.repmat(masses, 3, 1)), axis=1) / self._mass

  def _update_pose_estimate(self, gyro, orientation):
    """
    TODO: Finish documenting

    The CoM of all bodies has been updated at this point
    :param gryo:
    :param orientation: [numModules x 4] matrix of orientation
    """
    imu_frames = self._imu_frames
    l_leg = self._left_leg
    r_leg = self._right_leg
    l_arm = self._left_arm
    r_arm = self._right_arm

    # Update gyro values from current modules feedback
    # Transposing allows us to do calculations a bit easier below
    self._pose_gyros[:, :] = gyro.T

    # Update imu frames
    # Leg end effectors
    imu_frames[0] = l_leg.current_tip_fk
    imu_frames[1] = r_leg.current_tip_fk
    # Hip link output frames
    imu_frames[3] = l_leg.current_fk[1]
    imu_frames[5] = r_leg.current_fk[1]
    # Arm base bracket
    imu_frames[7] = l_arm.current_fk[1]
    imu_frames[11] = r_arm.current_fk[1]
    # Arm shoulder link
    imu_frames[8] = l_arm.current_fk[3]
    imu_frames[12] = r_arm.current_fk[3]
    # Arm elbow link
    imu_frames[9] = l_arm.current_fk[5]
    imu_frames[13] = r_arm.current_fk[5]

    q_rot = np.empty((3, 3), dtype=np.float64)
    rpy = np.empty(3, dtype=np.float64)
    rpy_modules = self._rpy

    for i in range(14):
      imu_frame = imu_frames[i][0:3, 0:3]
      # numpy arrays and matrices are different types, and you can't simply do
      # np_matrix * np_array
      # `np.inner` supports exactly what we are trying to do, though
      self._pose_gyros[0:3, i] = np.inner(imu_frame, self._pose_gyros[0:3, i])
      quaternion = orientation[i, 0:4]
      if math_func.any_nan(quaternion):
        rpy_modules[0:3, i] = np.nan
      else:
        # rotation matrix transpose is same as inverse since `rot` is in SO(3)
        q_rot = math_func.quat2rot(quaternion, q_rot)
        q_rot = q_rot*imu_frame.T
        rpy = math_func.rot2ea(q_rot, rpy)
        rpy_modules[0:3, i] = rpy

    self._pose_gyros_mean = np.nanmean(self._pose_gyros[:, self._imu_modules], axis=1)

  def _calculate_lean_angle(self):
    """
    TODO: Finish documenting

    The CoM of all individual bodies and pose estimate have been updated at this point
    """
    rpy_module = self._rpy
    imu_modules = self._imu_modules

    # {roll,pitch}_angle are in radians here
    roll_angle = np.nanmean(rpy_module[0, imu_modules])
    pitch_angle = np.nanmean(rpy_module[1, imu_modules])

    # Update roll (rotation matrix)
    math_func.rotate_x(roll_angle, output=self._roll_rot)
    # Update pitch (rotation matrix)
    math_func.rotate_y(pitch_angle, output=self._pitch_rot)

    self._roll_angle = math.degrees(roll_angle)
    self._pitch_angle = math.degrees(pitch_angle)

    T_pose = self._pose_transform
    T_pose[0:3, 0:3] = self._pitch_rot * self._roll_rot

    # rad/s
    self._feedback_lean_angle_velocity = self._pose_gyros_mean[1]

    # NOTE(rLinks234): MATLAB `leanR` is `self._pitch_rot` here

    # Gets the translation vector of the Legs' current end effectors
    l_leg_t = self._left_leg.current_tip_fk[0:3, 3]
    r_leg_t = self._right_leg.current_tip_fk[0:3, 3]
    ground_point = (l_leg_t+r_leg_t)*0.5
    line_com = np.inner(self._pitch_rot, self._com-ground_point.A1)
    self._height_com = np.linalg.norm(line_com)
    self._feedback_lean_angle = math.degrees(math.atan2(line_com[0], line_com[2]))

    # Update Igor center of mass
    self._com = np.inner(T_pose[0:3, 0:3], self._com)

    l_leg_coms = self._left_leg.current_coms
    r_leg_coms = self._right_leg.current_coms
    # Update CoM of legs based on current pose estimate from calculated lean angle
    for i in range(len(l_leg_coms)):
      l_leg_coms[i] = T_pose*l_leg_coms[i]
    for i in range(len(r_leg_coms)):
      r_leg_coms[i] = T_pose*r_leg_coms[i]

# ------------------------------------------------
# Actions
# ------------------------------------------------

  def _soft_startup(self):
    l_arm = self._left_arm
    r_arm = self._right_arm
    l_leg = self._left_leg
    r_leg = self._right_leg

    l_arm_i = l_arm.group_indices
    r_arm_i = r_arm.group_indices
    l_leg_i = l_leg.group_indices
    r_leg_i = r_leg.group_indices

    group = self._group
    group_feedback = self._group_feedback
    group_command = self._group_command

    group.send_feedback_request()
    group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
    self._group_feedback = group_feedback

    self._time_last[:] = group_feedback.receive_time
    positions = group_feedback.position

    l_arm_t = l_arm.create_home_trajectory(positions)
    r_arm_t = r_arm.create_home_trajectory(positions)
    l_leg_t = l_leg.create_home_trajectory(positions)
    r_leg_t = r_leg.create_home_trajectory(positions)

    start_time = time()
    t = 0.0

    t_pose = self._pose_transform
    gravity = -1.0*t_pose[2, 0:3]

    # Update time for chassis
    self._chassis.update_time()

    left_knee = 0.0
    right_knee = 0.0

    while t < 3.0:
      # Limit commands initially
      soft_start = min(t/3.0, 1.0)

      grav_comp_efforts = l_arm.get_grav_comp_efforts(positions, gravity)
      pos, vel, accel = l_arm_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, grav_comp_efforts, l_arm_i)

      grav_comp_efforts = r_arm.get_grav_comp_efforts(positions, gravity)
      pos, vel, accel = r_arm_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, grav_comp_efforts, r_arm_i)

      pos, vel, accel = l_leg_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, None, l_leg_i)
      left_knee = pos[1]

      pos, vel, accel = r_leg_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, None, r_leg_i)
      right_knee = pos[1]

      # Scale effort
      group_command.effort = soft_start*group_command.effort
      #show_command(group_cmd, t)
      group.send_command(group_command)
      #group.send_feedback_request()
      group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
      positions = group_feedback.position
      t = time()-start_time

    self._time_last[:] = group_feedback.receive_time
    self._left_leg._knee_angle = left_knee
    self._right_leg._knee_angle = -right_knee


  def _spin_once(self, bc):
    """

    :param bc:
    :type bc:  bool
    :return:
    """
    self._group_feedback = self._group.get_next_feedback(reuse_fbk=self._group_feedback)
    group_feedback = self._group_feedback
    group_command = self._group_command

    rel_time = time()-self._start_time
    soft_start = min(rel_time, 1.0)
    rx_time = group_feedback.receive_time

    if self._config.is_imitation:
      dt = 0.01
    else:
      dt = np.mean(rx_time-self._time_last)

    self._time_last[:] = rx_time

    positions = group_feedback.position
    commanded_positions = group_feedback.position_command
    velocities = group_feedback.velocity
    velocity_error = group_feedback.velocity_command-velocities
    gyro = group_feedback.gyro
    orientation = group_feedback.orientation

    if self._config.is_imitation:
      gyro[:, :] = 0.33
      orientation[:, 0] = 0.25
      orientation[:, 1] = 1.0
      orientation[:, 2:4] = 0.0

    self._update_com(positions, commanded_positions)
    self._update_pose_estimate(gyro, orientation)
    self._calculate_lean_angle()

    # ----------------------------
    # Value Critical section begin
    self._value_lock.acquire()

    # For now, we only use the left arm velocity values.
    # This is because we send the same values to both the left and right arm,
    # and also because the chassis trajectory only has 3 command entries for arm velocity values

    # The knee velocities are commanded by the user (joystick `OPTIONS`), and the event callback
    # sets the velocity on the Leg objects in response to user interaction
    user_commanded_knee_velocity = self._left_leg.user_commanded_knee_velocity
    user_commanded_grip_velocity = self._left_arm.user_commanded_grip_velocity

    self._chassis.update_trajectory(user_commanded_knee_velocity, user_commanded_grip_velocity)
    self._chassis.integrate_step(dt)

    calculated_knee_velocity = self._chassis.calculated_knee_velocity
    calculated_grip_velocity = self._chassis.calculated_grip_velocity

    self._left_leg.integrate_step(dt, calculated_knee_velocity)
    self._right_leg.integrate_step(dt, calculated_knee_velocity)
    self._left_arm.integrate_step(dt, positions, calculated_grip_velocity)
    self._right_arm.integrate_step(dt, positions, calculated_grip_velocity)

    self._chassis.update_velocity_controller(dt, velocities, self._wheel_radius,
                                             self._height_com, self._feedback_lean_angle_velocity,
                                             self._mass, self._feedback_lean_angle)

    if bc: # bc
      leanP = Igor.Lean_P
      leanI = Igor.Lean_I
      leanD = Igor.Lean_D

      l_wheel = group_command[0]
      r_wheel = group_command[1]

      p_effort = (leanP*self._chassis.lean_angle_error)+(leanI*self._chassis.lean_angle_error_cumulative)+(leanD*self._feedback_lean_angle_velocity)
      l_wheel.effort = p_effort*soft_start
      r_wheel.effort = -p_effort*soft_start

    # --------------------------------
    # Wheel Commands
    max_velocity = 10.0
    l_wheel_vel = min(max(self._chassis.calculated_yaw_velocity+self._chassis.velocity_feedforward, -max_velocity), max_velocity)
    r_wheel_vel = min(max(self._chassis.calculated_yaw_velocity-self._chassis.velocity_feedforward, -max_velocity), max_velocity)
    group_command[0].velocity = l_wheel_vel
    group_command[1].velocity = r_wheel_vel

    # ------------
    # Leg Commands

    # Roll Angle is in degrees, but Leg needs it to be in radians
    roll_angle = math.radians(self._roll_angle)

    self._left_leg.update_command(group_command, velocity_error, roll_angle, soft_start)
    self._right_leg.update_command(group_command, velocity_error, roll_angle, soft_start)

    # ------------
    # Arm Commands

    self._left_arm.update_command(group_command, positions, velocities, self._pose_transform, soft_start)
    self._right_arm.update_command(group_command, positions, velocities, self._pose_transform, soft_start)

    self._value_lock.release()
    # --------------------------
    # Value Critical section end

    ####
    # Debugging
    demo_gui.request_update()
    ####

    self._group.send_command(group_command)
    group_command.position = None
    group_command.velocity = None
    group_command.effort = None

# ------------------------------------------------
# Lifecycle functions
# ------------------------------------------------

  def _stop(self):
    """
    Stop running Igor. This happens once the user requests the demo to stop,
    or when the running application begins to shut down.

    This is only called by :meth:`__start`l_leg
    """
    if self._restart_flag:
      # Prepare for a restart
      # TODO
      pass
    else:
      # TODO
      print('Shutting down Igor...')

  def _start(self):
    """
    Main processing method. This runs on a background thread.
    """
    self._soft_startup()

    # Delay registering event handlers until now, so Igor
    # can start up without being interrupted by user commands.
    # View this function in `event_handlers.py` to see
    # all of the joystick event handlers registered
    register_igor_event_handlers(self, self._joy)

    self._start_time = time()
    self._state_lock.acquire()
    while self._continue():
      # We have `_state_lock` at this point. Access fields here before releasing lock
      bc = self._balance_controller_enabled
      self._state_lock.release()
      self._spin_once(not bc) # Swap for now
      self._state_lock.acquire()

    self._stop()

# ------------------------------------------------
# Initialization functions
# ------------------------------------------------

  def _find_joystick(self):
    group = self._group
    group_command = hebi.GroupCommand(group.size)

    if self._config.is_imitation:
      joy = DemoUtils.retry_on_error(get_first_joystick)
    else:
      on_error = funpart(on_error_find_joystick, group, group_command)
      joy = DemoUtils.retry_on_error(get_first_joystick, on_error)

    self._joy = joy

  def _load_gains(self):
    group = self._group

    # Bail out if group is imitation
    if self._config.is_imitation:
      return

    gains_command = hebi.GroupCommand(group.size)
    sleep(0.1)

    try:
      if self._has_camera:
        gains_command.read_gains(self._config.gains_xml)
      else:
        gains_command.read_gains(self._config.gains_no_camera_xml)
    except Exception as e:
      print('Warning: Could not load gains\nException: {0}'.format(e))
      return

    # Send gains multiple times
    for i in range(3):
      group.send_command(gains_command)
      sleep(0.1)

  def __init__(self, has_camera=False, config=None):
    if config == None:
      self._config = Igor2Config()
    else:
      assert_type(config, Igor2Config, 'config')
      self._config = config

    if has_camera:
      num_dofs = 15
    else:
      num_dofs = 14

    # The joystick interface
    self._joy = None

    # ------------
    # Group fields
    # These group message objects are only used by the proc thread
    # They are NOT thread safe.
    self._group = None
    self._group_command = None
    self._group_feedback = None
    self._group_info = None

    self._proc_thread = None

    # ----------------
    # Parameter fields
    self._has_camera = has_camera
    self._joy_dead_zone = 0.06
    self._wheel_radius = 0.100
    self._wheel_base = 0.43

    # ------------
    # State fields
    from threading import Lock
    self._state_lock = Lock()
    self._started = False
    self._balance_controller_enabled = True
    self._quit_flag = False
    self._restart_flag = False
    self._time_last = np.empty(num_dofs, dtype=np.float64)
    value_lock = Lock()
    self._value_lock = value_lock
    self._start_time = -1.0

    # ---------------------
    # Kinematic body fields
    chassis = Chassis(value_lock)
    l_leg = Leg(value_lock, 'Left', [2, 3])
    r_leg = Leg(value_lock, 'Right', [4, 5])
    l_arm = Arm(value_lock, 'Left', [6, 7, 8, 9])
    r_arm = Arm(value_lock, 'Right', [10, 11, 12, 13])

    self._chassis = chassis
    self._left_leg = l_leg
    self._right_leg = r_leg
    self._left_arm = l_arm
    self._right_arm = r_arm

    # ---------------------------
    # Cached fields for body mass
    self._mass = l_leg.mass + r_leg.mass + l_arm.mass + r_arm.mass + chassis.mass

    self._masses = np.array([l_leg.mass, r_leg.mass,
                             l_arm.mass, r_arm.mass, chassis.mass],
                            dtype=np.float64)

    # --------------------------------------------
    # Cached fields for center of mass calculation
    self._com = np.empty(3, dtype=np.float64)
    self._coms = np.empty((3, 5), dtype=np.float64)
    # This CoM is constant
    self._coms[0:3, 4] = chassis.com

    # ----------------------------------
    # Cached fields for pose estimation
    imu_frames = [np.identity(4,dtype=np.float64)]*15
    # These frames are constant
    imu_frames[2] = l_leg._robot.base_frame
    imu_frames[4] = r_leg._robot.base_frame
    imu_frames[6] = l_arm._robot.base_frame
    imu_frames[10] = r_arm._robot.base_frame
    self._imu_frames = imu_frames

    # All of the leg modules, plus the wheel modules
    self._imu_modules = [0, 1, 2, 3, 4, 5]

    self._pose_gyros = np.empty((3, num_dofs), dtype=np.float64)
    self._pose_gyros_mean = None

    # Roll-Pitch-Yaw
    self._rpy = np.empty((3, num_dofs), dtype=np.float64)

    self._pose_transform = np.identity(4, dtype=np.float64)

    # ----------------------------------------
    # Cached fields for lean angle calculation
    self._roll_angle = 0.0
    self._roll_rot = np.empty((3, 3), dtype=np.float64)
    self._pitch_angle = 0.0
    self._pitch_rot = np.empty((3, 3), dtype=np.float64)
    self._feedback_lean_angle_velocity = 0.0
    # These are used for chassis velocity controller
    self._height_com = 0.0
    self._feedback_lean_angle = 0.0

# ------------------------------------------------
# Public Interface
# ------------------------------------------------

  def start(self):
    """
    Start running Igor with the provided configuration.
    This can safely be called multiple times
    but will do nothing after the first invocation.

    All processing will be done on a background thread which is spawned
    in this method.
    
    TODO: Finish documenting
    """
    self._state_lock.acquire()
    if self._started:
      self._state_lock.release()
      return
    
    group = create_group(self._config, self._has_camera)
    group.command_lifetime = 500
    group.feedback_frequency = 100.0

    self._group = group
    self._group_command = hebi.GroupCommand(group.size)
    self._group_feedback = hebi.GroupFeedback(group.size)
    self._group_info = hebi.GroupInfo(group.size)

    self._find_joystick()
    self._load_gains()

    from threading import Condition, Lock, Thread
    start_condition = Condition(Lock())

    def start_routine(start_condition):
      start_condition.acquire()

      # Calling thread holds `__state_lock` AND
      # is also blocked until we call `start_condition.notify_all()`
      # So we can safely modify this here.
      self._started = True
      
      # Allow the calling thread to continue
      start_condition.notify_all()
      start_condition.release()

      self._start()

    self._proc_thread = Thread(target=start_routine, name='Igor II Controller',
                                args = (start_condition,))
    
    # We will have started the thread before returning,
    # but make sure the function has begun running before
    # we release the `__state_lock` and return
    start_condition.acquire()
    self._proc_thread.start()
    start_condition.wait()
    start_condition.release()

    self._state_lock.release()

  def request_stop(self):
    """
    Send a request to stop the demo.
    If the demo has not been started, this method raises an exception.

    :raises RuntimeError: if `start()` has not been called
    """
    self._state_lock.acquire()
    self._ensure_started()
    self._quit_flag = True
    self._state_lock.release()

  def request_restart(self):
    """
    Send a request to restart the demo.
    If the demo has not been started, this method raises an exception.

    :raises RuntimeError: if `start()` has not been called
    """
    self._state_lock.acquire()
    self._ensure_started()
    self._restart_flag = True
    self._quit_flag = True
    self._state_lock.release()

  def set_balance_controller_state(self, state):
    """
    Set the state of the balance controller.
    If the demo has not been started, this method raises an exception.

    :param state: ``True`` for enabled, ``False`` for disabled
    :type state:  bool

    :raises TypeError:    if `state` is not bool
    :raises RuntimeError: if `start()` has not been called
    """
    assert_type(state, bool, 'state')
    self._state_lock.acquire()
    self._ensure_started()
    self._balance_controller_enabled = state
    self._state_lock.release()

# ------------------------------------------------
# Properties
# ------------------------------------------------

  @property
  def group(self):
    return self._group

  @property
  def joystick_dead_zone(self):
    return self._joy_dead_zone

  @property
  def has_camera(self):
    return self._has_camera

  @property
  def started(self):
    self._state_lock.acquire()
    val = self._started
    self._state_lock.release()
    return val

  @property
  def wheel_radius(self):
    return self._wheel_radius

  @property
  def wheel_base(self):
    return self._wheel_base

  @property
  def mass(self):
    return self._mass

  @property
  def left_arm(self):
    return self._left_arm

  @property
  def right_arm(self):
    return self._right_arm

  @property
  def left_leg(self):
    return self._left_leg

  @property
  def right_leg(self):
    return self._right_leg

  @property
  def chassis(self):
    return self._chassis

