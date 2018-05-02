import math
import numpy as np
import numpy.matlib as matlib

from .config import Igor2Config
from .event_handlers import register_igor_event_handlers
from .igor_utils import (create_group, find_joystick, is_main_thread_active,
                         load_gains, set_command_subgroup_pve)

# kits.util._internal.type_utils
from ..util._internal.type_utils import assert_instance, assert_length, assert_type
from ..util import math_func
import hebi

from time import sleep, time


# ------------------------------------------------------------------------------
# Base Classes
# ------------------------------------------------------------------------------


class BaseBody(object):
  """
  Base class for all body components of Igor
  """

  def __init__(self, val_lock, mass=0.0, com=[0.0, 0.0, 0.0]):
    self.__val_lock = val_lock
    self._mass = mass
    self._com = np.asarray(com, dtype=np.float64)

  def _set_mass(self, mass):
    self._mass = mass

  def _set_com(self, com):
    self._com = np.asarray(com, dtype=np.float64)

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
    :rtype:  np.array
    """
    return self._com


class PeripheralBody(BaseBody):
  """
  Base class for all peripheral body components of Igor (e.g., legs and arms)
  """

  def __init__(self, val_lock, group_indices, name, mass=None, com=None):
    if mass != None and com != None:
      super(PeripheralBody, self).__init__(val_lock, mass, com)
    else:
      super(PeripheralBody, self).__init__(val_lock)

    self._group_indices = group_indices
    # TODO: Maybe have these be matrices instead of arrays?
    self._fbk_position = np.zeros(len(group_indices), dtype=np.float64)
    self._fbk_position_cmd = np.zeros(len(group_indices), dtype=np.float64)
    self._fbk_velocity = np.zeros(len(group_indices), dtype=np.float32)
    self._fbk_velocity_err = np.zeros(len(group_indices), dtype=np.float32)

    self._xyz_error = np.zeros((3, 1), dtype=np.float64)
    self._pos_error = np.zeros((6, 1), dtype=np.float64)
    self._vel_error = np.zeros((6, 1), dtype=np.float32)
    self._impedance_err = np.zeros((6, 1), dtype=np.float64)
    self._impedance_torque = np.zeros((len(group_indices), 1), dtype=np.float64)

    self._name = name
    self._kin = hebi.robot_model.RobotModel()
    self._home_angles = None
    self._masses_t = None

    self._current_coms = None
    self._current_fk = None
    self._current_tip_fk = None
    self._current_j_actual = None
    self._current_j_actual_f = None
    self._current_j_expected = None
    self._current_j_expected_f = None
    self._current_xyz = None

  def _set_masses_t(self, masses):
    self._masses_t = masses

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
    home_angles = np.asarray(value, dtype=np.float64)
    assert_length(home_angles, len(self._group_indices), 'home_angles')
    self._home_angles = home_angles

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
    TODO: Document
    :return:
    :rtype:
    """
    return self._current_coms

  @property
  def current_fk(self):
    """
    TODO: Document
    :return:
    :rtype:
    """
    return self._current_fk

  @property
  def current_tip_fk(self):
    """
    TODO: Document
    :return:
    :rtype:
    """
    return self._current_tip_fk

  @property
  def current_j_actual(self):
    """
    TODO: Document
    :return:
    :rtype:
    """
    return self._current_j_actual

  @property
  def current_j_expected(self):
    """
    TODO: Document
    :return:
    :rtype:
    """
    return self._current_j_expected

  def on_feedback_received(self, position, position_command, velocity, velocity_error):
    np.copyto(self._fbk_position, position[self._group_indices])
    np.copyto(self._fbk_position_cmd, position_command[self._group_indices])
    np.copyto(self._fbk_velocity, velocity[self._group_indices])
    np.copyto(self._fbk_velocity_err, velocity_error[self._group_indices])

  def get_grav_comp_efforts(self, positions, gravity):
    """
    TODO: Document

    :param positions: TODO
    :type positions:  np.array
    :param gravity:   TODO
    :type gravity:    np.array

    :return: TODO
    :rtype:  np.array
    """
    kin = self._kin
    positions = positions[self._group_indices]
    return math_func.get_grav_comp_efforts(kin, positions, gravity)

  def create_home_trajectory(self, positions, duration=3.0):
    """
    Create a trajectory from the current pose to the home pose.
    This is used on soft startup
    ^^^ Is this the correct usage of "pose" ???

    :param positions: TODO
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
    num_joints = len(self._group_indices)
    num_waypoints = 2
    dim = (num_joints, num_waypoints)

    current_positions = positions[self._group_indices]
    home_angles = self._home_angles

    times = np.array([0.0, duration], dtype=np.float64)
    pos = np.empty(dim, dtype=np.float64)
    vel = np.zeros(dim, dtype=np.float64)
    accel = vel

    pos[:, 0] = current_positions
    pos[:, 1] = home_angles

    return hebi.trajectory.create_trajectory(times, pos, vel, accel)

  def update_position(self):
    """
    Upate kinematics from feedback
    TODO: document and CLEAN UP

    :param positions:           TODO
    :type positions:            np.array
    :param commanded_positions: TODO
    :type commanded_positions:  np.array
    """
    positions = self._fbk_position
    commanded_positions = self._fbk_position_cmd

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
    self._current_j_actual_f = np.matrix(jacobian_actual, np.float32)
    self._current_j_expected = jacobian_expected
    self._current_j_expected_f = np.matrix(jacobian_expected, np.float32)
    self._current_xyz = xyz
    self._set_com(com)


# ------------------------------------------------------------------------------
# Kinematics Classes
# ------------------------------------------------------------------------------


class Chassis(BaseBody):
  """
  Class representing the chassis of Igor
  """

  Velocity_P = 15.0
  """
  The proportional term of the chassis velocity controller
  """

  Velocity_I = 0.1
  """
  The integral term of the chassis velocity controller
  """

  Velocity_D = .3
  """
  The derivative term of the chassis velocity controller
  """

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
    """
    TODO: Document
    """
    self._time_s = time()

  def update_trajectory(self, user_commanded_knee_velocity, user_commanded_grip_velocity):
    """
    TODO: Document

    :param user_commanded_knee_velocity: The knee velocity calculated from joystick or user input
    :type user_commanded_knee_velocity:  float
    :param user_commanded_grip_velocity: The grip velocity (x,y,z) calculated from joystick or user input
    :type user_commanded_grip_velocity:  np.array
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
    TODO: Document
    Called by Igor. User should not call this directly.

    :param dt: timestep used in integrating
    :type dt:  float
    """
    self._hip_pitch = self._hip_pitch+self._hip_pitch_velocity*dt

  def update_velocity_controller(self, dt, velocities, wheel_radius,
                                 height_com, fbk_lean_angle_vel,
                                 robot_mass, fbk_lean_angle):
    """
    TODO: Document

    :param dt:
    :param velocities:
    :param wheel_radius:
    :param height_com:
    :param fbk_lean_angle_vel:
    :param robot_mass:
    :param fbk_lean_angle:
    """
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
    :return: TODO
    :rtype:  float
    """
    return self._velocities[0, 1]

  @property
  def user_commanded_yaw_velocity(self):
    """
    :return: TODO
    :rtype:  float
    """
    return self._velocities[1, 1]

  @property
  def user_commanded_knee_velocity(self):
    """
    :return: TODO
    :rtype:  float
    """
    return self._velocities[2, 1]

# ------------------------------------------------------------------------------
# Calculated Properties
# ------------------------------------------------------------------------------

  @property
  def calculated_lean_angle(self):
    """
    The calculated lean angle of the chassis. This is calculated by the
    velocity PID controller and is dependent on the velocity error
    and the lean feedforward term

    TODO: check if this is actually in degrees
    :return: The calculated lean angle (in degrees)
    :rtype:  float
    """
    return self._calculated_lean_angle

  @property
  def calculated_directional_velocity(self):
    """
    TODO: Document
    TODO: What are the units here? m/s?

    :return: the calculated directional velocity
    :float:  float
    """
    return self._velocities[0, 0]

  @property
  def calculated_yaw_velocity(self):
    """
    TODO: Document
    TODO: What are the units here? m/s?

    :return: the calculated yaw velocity
    :float:  float
    """
    return self._velocities[1, 0]

  @property
  def calculated_knee_velocity(self):
    """
    TODO: Document
    TODO: What are the units here? m/s?

    :return: the calculated knee velocity (in rad/s)
    :float:  float
    """
    return self._velocities[2, 0]

  @property
  def calculated_grip_velocity(self):
    """
    TODO: Document
    TODO: What are the units here? m/s?

    :return:
    :float:  np.array
    """
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
  """
  TODO: Document
  """

  spring_gains = np.matrix([100.0, 10.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  """
  TODO: Document
  """

  Jacobian_Determinant_Threshold = 0.010
  """
  The lower threshold allowed for the determinant calculation of the jacobians.
  Anything below this will be considered at or near a singularity.
  """

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
    self._joint_velocities = np.zeros(4, np.float32)
    self._joint_efforts = np.zeros(4, np.float32)
    self._user_commanded_wrist_velocity = 0.0

    self._grav_comp_torque = np.zeros((len(group_indices), 1))

    # Additionally, calculate determinant of jacobians
    self._current_det_actual = None
    self._current_det_expected = None

  def integrate_step(self, dt, calculated_grip_velocity):
    """
    Called by Igor. User should not call this directly.

    :param dt:
    :type dt:                        float
    :param positions:
    :type positions:                 np.array
    :param calculated_grip_velocity:
    :type calculated_grip_velocity:  np.array
    """
    positions = self._fbk_position
    # Make end effector velocities mirrored in Y
    adjusted_grip_velocity = calculated_grip_velocity.copy()
    adjusted_grip_velocity[1] = self._direction*adjusted_grip_velocity[1]

    np.multiply(adjusted_grip_velocity, dt, out=adjusted_grip_velocity)
    np.add(self._grip_pos, adjusted_grip_velocity, out=self._new_grip_pos)
    robot = self._robot
    xyz_objective = hebi.robot_model.endeffector_position_objective(self._new_grip_pos)
    new_arm_joint_angs = robot.solve_inverse_kinematics(positions, xyz_objective)

    jacobian_new = robot.get_jacobian_end_effector(new_arm_joint_angs)[0:3, 0:3]
    det_J_new = abs(np.linalg.det(jacobian_new))

    if (self._current_det_expected < Arm.Jacobian_Determinant_Threshold) and (det_J_new < self._current_det_expected):
      # Near singularity - don't command arm towards it
      self._joint_velocities[0:3] = 0.0
    else:
      try:
        self._joint_velocities[0:3] = np.linalg.solve(self.current_j_actual[0:3, 0:3], self._user_commanded_grip_velocity)
        self._joint_angles[0:3] = new_arm_joint_angs[0:3]
        np.copyto(self._grip_pos, self._new_grip_pos)
      except np.linalg.LinAlgError as lin:
        # This may happen still sometimes
        self._joint_velocities[0:3] = 0.0

    wrist_vel = self._direction*self._user_commanded_wrist_velocity
    self._joint_velocities[3] = self._joint_velocities[1]+self._joint_velocities[2]+wrist_vel
    self._joint_angles[3] = self._joint_angles[3]+(self._joint_velocities[3]*dt)

  @property
  def current_det_actual(self):
    """
    TODO: Document

    :return:
    :rtype:  float
    """
    return self._current_det_actual

  @property
  def current_det_expected(self):
    """
    TODO: Document

    :return:
    :rtype:  float
    """
    return self._current_det_expected

  @property
  def user_commanded_grip_velocity(self):
    """
    TODO: Document

    :return:
    :rtype:  float
    """
    return self._user_commanded_grip_velocity

  @property
  def user_commanded_wrist_velocity(self):
    """
    TODO: Document

    :return:
    :rtype:  float
    """
    return self._user_commanded_wrist_velocity

  @property
  def grip_position(self):
    """
    TODO: Document

    :return:
    :rtype:  np.array
    """
    return self._grip_pos

  def update_position(self):
    """
    TODO: Document

    :param positions:
    :param commanded_positions:
    :return:
    """
    super(Arm, self).update_position()
    self._current_det_actual = np.linalg.det(self._current_j_actual[0:4, 0:4])
    self._current_det_expected = np.linalg.det(self._current_j_expected[0:4, 0:4])

  def update_command(self, group_command, pose, soft_start):
    """
    TODO: Document

    :param group_command:
    :param positions:
    :param velocities:
    :param pose:
    :param soft_start:
    """

    commanded_positions = self._joint_angles
    commanded_velocities = self._joint_velocities

    positions = self._fbk_position
    velocities = self._fbk_velocity

    # Positions and velocities are already known at this point - don't need to calculate them

    # ----------------
    # Calculate effort
    np.subtract(np.asmatrix(self._grip_pos).T, self.current_tip_fk[0:3, 3], out=self._xyz_error)
    np.copyto(self._pos_error[0:3], self._xyz_error)
    np.subtract(commanded_velocities, velocities, out=self._vel_error[0:4].ravel())
    np.dot(self._current_j_actual_f, self._vel_error[0:4], out=self._vel_error)

    np.multiply(Arm.spring_gains, self._pos_error, out=self._pos_error)
    np.multiply(Arm.damper_gains, self._vel_error, out=self._vel_error)
    np.add(self._pos_error, self._vel_error, out=self._impedance_err)
    np.dot(self.current_j_actual.T, self._impedance_err, out=self._impedance_torque)
    #math_func.get_grav_comp_efforts(self._robot, positions, -pose[2, 0:3], self._grav_comp_torque)
    np.copyto(self._grav_comp_torque.ravel(), math_func.get_grav_comp_efforts(self._robot, positions, -pose[2, 0:3]))

    np.multiply(soft_start, self._impedance_torque.ravel(), out=self._joint_efforts)
    np.add(self._joint_efforts, self._grav_comp_torque.ravel(), out=self._joint_efforts)
    effort = self._joint_efforts

    # Send commands
    idx = 0
    for i in self.group_indices:
      group_command[i].position = commanded_positions[idx]
      group_command[i].velocity = commanded_velocities[idx]
      group_command[i].effort = effort[idx]
      idx = idx + 1

  def set_x_velocity(self, value):
    """
    TODO: Document
    TODO: What units are velocity in here? rad/s or m/s?

    :param value: the x velocity of the arm
    :type value:  float
    """
    self.acquire_value_lock()
    self._user_commanded_grip_velocity[0] = value
    self.release_value_lock()

  def set_y_velocity(self, value):
    """
    TODO: Document
    TODO: What units are velocity in here? rad/s or m/s?

    :param value: the y velocity of the arm
    :type value:  float
    """
    self.acquire_value_lock()
    self._user_commanded_grip_velocity[1] = value
    self.release_value_lock()

  def set_z_velocity(self, value):
    """
    TODO: Document
    TODO: What units are velocity in here? rad/s or m/s?

    :param value: the z velocity of the arm
    :type value:  float
    """
    self.acquire_value_lock()
    self._user_commanded_grip_velocity[2] = value
    self.release_value_lock()

  def set_wrist_velocity(self, value):
    """
    TODO: Document

    :param value: the velocity of the wrist (in rad/s)
    :type value:  float
    """
    self.acquire_value_lock()
    self._user_commanded_wrist_velocity = value
    self.release_value_lock()


class Leg(PeripheralBody):
  """
  Represents a leg (and wheel)
  """

  damper_gains = np.matrix([2.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  """
  TODO: Document
  """

  spring_gains = np.matrix([400.0, 0.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  """
  TODO: Document
  """

  roll_gains = np.matrix([0.0, 0.0, 10.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  """
  TODO: Document
  """

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
    self._e_term = np.empty((6, 1), dtype=np.float64)

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
    self._hip_angle = (np.pi+self._knee_angle)*0.5

  def update_position(self):
    """
    TODO: Document
    """
    super(Leg, self).update_position()
    self._current_cmd_tip_fk = self._robot.get_forward_kinematics('endeffector', self._fbk_position_cmd)[0]

  def update_command(self, group_command, roll_angle, soft_start):
    """
    TODO: Document

    :param group_command:
    :param vel_error:
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
    np.subtract(self._current_cmd_tip_fk[0:3, 3], self.current_tip_fk[0:3, 3], out=self._xyz_error)
    np.copyto(self._pos_error[0:3], self._xyz_error)
    np.dot(self._current_j_actual_f, np.asmatrix(self._fbk_velocity_err).T, out=self._vel_error)
    roll_sign = self._direction

    np.multiply(Leg.spring_gains, self._pos_error, out=self._pos_error)
    np.multiply(Leg.damper_gains, self._vel_error, out=self._vel_error)
    np.multiply(Leg.roll_gains, roll_sign*roll_angle, out=self._e_term)
    np.add(self._pos_error, self._vel_error, out=self._impedance_err)
    np.add(self._impedance_err, self._e_term, out=self._impedance_err)
    np.dot(self.current_j_actual.T, self._impedance_err, out=self._impedance_torque)
    np.multiply(self._impedance_torque, soft_start, out=self._impedance_torque)

    hip_cmd.effort = self._impedance_torque[0, 0]
    knee_cmd.effort = self._impedance_torque[1, 0]

  @property
  def hip_angle(self):
    """
    TODO: Document
    :return: The current hip position (in radians)
    :rtype:  float
    """
    return self._hip_angle

  @property
  def knee_angle(self):
    """
    TODO: Document
    :return: The current knee angle (in radians)
    :rtype:  float
    """
    return self._knee_angle

  @property
  def user_commanded_knee_velocity(self):
    """
    TODO: Document
    :return: (in rad/s)
    :rtype:  float
    """
    return self._user_commanded_knee_velocity

  def set_knee_velocity(self, vel):
    """
    TODO: Document
    :param vel: (in rad/s)
    :type vel:  float
    """
    self.acquire_value_lock()
    self._user_commanded_knee_velocity = vel
    self.release_value_lock()


# ------------------------------------------------------------------------------
# Debugging stuff
# ------------------------------------------------------------------------------

#from . import demo_gui

# ------------------------------------------------------------------------------
# Igor Class
# ------------------------------------------------------------------------------


import threading
import cProfile
class ProfiledThread(threading.Thread):
  def run(self):
    profiler = cProfile.Profile()
    try:
      return profiler.runcall(threading.Thread.run, self)
    finally:
      profiler.dump_stats('igor-procthread-%d.profile' % (self.ident,))


class Igor(object):

  Lean_P = 1.0
  """
  TODO: Document
  """

  Lean_I = 20.0
  """
  TODO: Document
  """

  Lean_D = 10.0
  """
  TODO: Document
  """

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

  def _update_com(self):
    """
    TODO: Document
    :param positions:
    :param commanded_positions:
    """
    l_arm = self._left_arm
    r_arm = self._right_arm
    l_leg = self._left_leg
    r_leg = self._right_leg

    l_arm.update_position()
    r_arm.update_position()
    l_leg.update_position()
    r_leg.update_position()

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
    np.copyto(imu_frames[0], l_leg.current_tip_fk)
    np.copyto(imu_frames[1] , r_leg.current_tip_fk)
    # Hip link output frames
    np.copyto(imu_frames[3], l_leg.current_fk[1])
    np.copyto(imu_frames[5], r_leg.current_fk[1])
    # Arm base bracket
    np.copyto(imu_frames[7], l_arm.current_fk[1])
    np.copyto(imu_frames[11], r_arm.current_fk[1])
    # Arm shoulder link
    np.copyto(imu_frames[8], l_arm.current_fk[3])
    np.copyto(imu_frames[12], r_arm.current_fk[3])
    # Arm elbow link
    np.copyto(imu_frames[9], l_arm.current_fk[5])
    np.copyto(imu_frames[13], r_arm.current_fk[5])

    q_rot = np.empty((3, 3), dtype=np.float64)
    rpy_modules = self._rpy
    pose_tmp = self._pose_tmp
    quaternion_tmp = self._quaternion_tmp

    for i in range(14):
      imu_frame = imu_frames[i][0:3, 0:3]
      # numpy arrays and matrices are different types, and you can't simply do
      # np_matrix * np_array
      # `np.inner` supports exactly what we are trying to do, though
      np.dot(imu_frame, self._pose_gyros[0:3, i], out=pose_tmp)
      self._pose_gyros[0:3, i] = pose_tmp
      np.copyto(quaternion_tmp, orientation[i, 0:4])
      if math_func.any_nan(quaternion_tmp):
        rpy_modules[0:3, i] = np.nan
      else:
        # rotation matrix transpose is same as inverse since `rot` is in SO(3)
        math_func.quat2rot(quaternion_tmp, q_rot)
        np.matmul(q_rot, imu_frame.T, out=q_rot)
        math_func.rot2ea(q_rot, rpy_modules[0:3, i])

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
    np.matmul(self._pitch_rot, self._roll_rot, out=self._T_pose_rot)

    # rad/s
    self._feedback_lean_angle_velocity = self._pose_gyros_mean[1]

    # NOTE: MATLAB `leanR` is `self._pitch_rot` here

    # Gets the translation vector of the Legs' current end effectors
    np.add(self._left_leg.current_tip_fk[0:3, 3].A1,
           self._left_leg.current_tip_fk[0:3, 3].A1,
           out=self._ground_point)
    np.multiply(self._ground_point, 0.5, out=self._ground_point)
    np.dot(self._pitch_rot,
             np.subtract(self._com, self._ground_point),
             out=self._line_com)
    self._height_com = np.linalg.norm(self._line_com)
    self._feedback_lean_angle = math.degrees(math.atan2(self._line_com[0], self._line_com[2]))

    # Update Igor center of mass
    np.dot(self._T_pose_rot, self._com, out=self._com)

    T_pose[0:3, 0:3] = self._T_pose_rot
    l_leg_coms = self._left_leg.current_coms
    r_leg_coms = self._right_leg.current_coms
    # Update CoM of legs based on current pose estimate from calculated lean angle
    for i in range(len(l_leg_coms)):
      np.matmul(T_pose, l_leg_coms[i], out=l_leg_coms[i])
    for i in range(len(r_leg_coms)):
      np.matmul(T_pose, r_leg_coms[i], out=r_leg_coms[i])

# ------------------------------------------------
# Actions
# ------------------------------------------------

  def _soft_startup(self):
    """
    TODO: Document
    """
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

    positions = self._current_position

    group.send_feedback_request()
    group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
    self._group_feedback = group_feedback

    self._time_last[:] = group_feedback.receive_time
    group_feedback.get_position(positions)

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
      group.send_command(group_command)
      group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
      group_feedback.get_position(positions)
      t = time()-start_time

    self._time_last[:] = group_feedback.receive_time
    self._left_leg._knee_angle = left_knee
    self._right_leg._knee_angle = -right_knee


  def _spin_once(self, bc):
    """
    TODO: Document

    :param bc: Denotes whether the balance controller is enabled at this time
    :type bc:  bool
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

    np.copyto(self._time_last, rx_time)

    # ------------------------------------------------------
    # These fields are cached to avoid instantiating objects
    positions = self._current_position
    commanded_positions = self._current_position_command
    velocities = self._current_velocity
    velocity_commands = self._current_velocity_command
    velocity_error = self._current_velocity_error

    group_feedback.get_position(positions)
    group_feedback.get_position_command(commanded_positions)
    group_feedback.get_velocity(velocities)
    group_feedback.get_velocity_command(velocity_commands)
    np.subtract(velocity_commands, velocities, out=velocity_error)

    self._left_leg.on_feedback_received(positions, commanded_positions, velocities, velocity_error)
    self._right_leg.on_feedback_received(positions, commanded_positions, velocities, velocity_error)
    self._left_arm.on_feedback_received(positions, commanded_positions, velocities, velocity_error)
    self._right_arm.on_feedback_received(positions, commanded_positions, velocities, velocity_error)

    # TODO: cache these too
    gyro = group_feedback.gyro
    orientation = group_feedback.orientation

    if self._config.is_imitation:
      gyro[:, :] = 0.33
      orientation[:, 0] = 0.25
      orientation[:, 1] = 1.0
      orientation[:, 2:4] = 0.0

    self._update_com()
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
    self._left_arm.integrate_step(dt, calculated_grip_velocity)
    self._right_arm.integrate_step(dt, calculated_grip_velocity)

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
      effort = p_effort*soft_start
      l_wheel.effort = effort
      r_wheel.effort = -effort

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

    self._left_leg.update_command(group_command, roll_angle, soft_start)
    self._right_leg.update_command(group_command, roll_angle, soft_start)

    # ------------
    # Arm Commands

    self._left_arm.update_command(group_command, self._pose_transform, soft_start)
    self._right_arm.update_command(group_command, self._pose_transform, soft_start)

    self._value_lock.release()
    # --------------------------
    # Value Critical section end

    ####
    # Debugging
    #demo_gui.request_update()
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
      duration = self._stop_time - self._start_time
      tics = float(self._num_spins)
      avg_frequency = tics/duration
      print('Ran for: {0} seconds.'.format(duration))
      print('Average processing frequency: {0} Hz'.format(avg_frequency))

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
      self._spin_once(bc)
      self._num_spins = self._num_spins + 1
      self._state_lock.acquire()

    self._stop_time = time()
    self._stop()

# ------------------------------------------------
# Initialization functions
# ------------------------------------------------

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
    self._stop_time = -1.0
    self._num_spins = 0

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
    self._line_com = np.empty(3, dtype=np.float64)
    self._ground_point = np.empty(3, dtype=np.float64)
    self._feedback_lean_angle = 0.0

    self._pose_tmp = np.empty(3, dtype=np.float64)
    self._quaternion_tmp = np.empty(4, dtype=np.float64)
    self._T_pose_rot = np.empty((3, 3), dtype=np.float64)

    # --------------------------
    # Cached fields for feedback
    self._current_position = np.empty(num_dofs, dtype=np.float64)
    self._current_position_command = np.empty(num_dofs, dtype=np.float64)
    self._current_velocity = np.empty(num_dofs, dtype=np.float32)
    self._current_velocity_command = np.empty(num_dofs, dtype=np.float32)
    self._current_velocity_error = np.empty(num_dofs, dtype=np.float32)

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

    self._joy = find_joystick(self)
    load_gains(self)

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

    self._proc_thread = ProfiledThread(target=start_routine, name='Igor II Controller',
                                args = (start_condition,))
    
    # We will have started the thread before returning,
    # but make sure the function has begun running before
    # we release the `_state_lock` and return
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
  def config(self):
    """
    The configuration of this Igor instance
    :rtype: Igor2Config
    """
    return self._config

  @property
  def group(self):
    """
    The HEBI group containing the Igor modules
    """
    return self._group

  @property
  def joystick_dead_zone(self):
    """
    The deadzone of the joystick used to control Igor
    TODO: Document
    :rtype: float
    """
    return self._joy_dead_zone

  @property
  def has_camera(self):
    """
    TODO: Document
    :rtype: bool
    """
    return self._has_camera

  @property
  def started(self):
    """
    TODO: Document
    :rtype: bool
    """
    self._state_lock.acquire()
    val = self._started
    self._state_lock.release()
    return val

  @property
  def wheel_radius(self):
    """
    The radius (in meters) of the wheels on this Igor instance
    :rtype: float
    """
    return self._wheel_radius

  @property
  def wheel_base(self):
    """
    TODO: Document
    :rtype: float
    """
    return self._wheel_base

  @property
  def mass(self):
    """
    The total weight (in kilograms) of this Igor instance
    :rtype: float
    """
    return self._mass

  @property
  def left_arm(self):
    """
    TODO: Document
    :rtype: Arm
    """
    return self._left_arm

  @property
  def right_arm(self):
    """
    TODO: Document
    :rtype: Arm
    """
    return self._right_arm

  @property
  def left_leg(self):
    """
    TODO: Document
    :rtype: Leg
    """
    return self._left_leg

  @property
  def right_leg(self):
    """
    TODO: Document
    :rtype: Leg
    """
    return self._right_leg

  @property
  def chassis(self):
    """
    TODO: Document
    :rtype: Chassis
    """
    return self._chassis

