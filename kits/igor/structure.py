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

  def _create_trajectory(self):
    return hebi.trajectory.create_trajectory(self._traj_times,
                                             self._velocities,
                                             self._accels,
                                             self._jerks)

  def __init__(self, val_lock):
    super(Chassis, self).__init__(val_lock, mass=6.0, com=[0.0, 0.0, 0.10+0.3])
    self._dir_vel = 0.0
    self._yaw_vel = 0.0
    self._min_ramp_time = 0.5
    self._hip_pitch = 0.0
    self._hip_pitch_vel = 0.0

    self._vel_ff = 0.0
    self._vel_error = 0.0
    self._vel_error_cum = 0.0
    self._fbk_chassis_vel_last = 0.0
    self._cmd_chassis_vel_last = 0.0
    self._lean_angle_error = 0.0
    self._lean_angle_error_cum = 0.0

    # ---------------
    # Trajectory data
    num_cmds = 6

    self._traj_times = np.array([0.0, self._min_ramp_time], dtype=np.float64)

    # Column[0]: `chassisVelNow` in MATLAB
    # Column[1]: `chassisVelCmd` in MATLAB
    self._velocities = np.zeros((num_cmds, 2), dtype=np.float64)

    # Column[0]: `chassisAccNow` in MATLAB
    # Column[1]: Always 0
    self._accels = np.zeros((num_cmds, 2), dtype=np.float64)

    # Column[0]: `chassisJerkNow` in MATLAB
    # Column[1]: Always 0
    self._jerks = np.zeros((num_cmds, 2), dtype=np.float64)

    self._trajectory = self._create_trajectory()
    self._time_s = None

  def update_time(self):
    self._time_s = time()

  def update_trajectory(self, knee_velocity, arm_velocity):
    """
    TODO: Document
    """
    # [ <chassis directional velocity>, <chassis yaw velocity>, <knee velocity>, <armX_v>, <armY_v>, <armZ_v> ]
    vel = self._velocities
    acc = self._accels
    jrk = self._jerks

    time_now = time()
    t = time_now - self._time_s
    self._time_s = time_now

    vel_now, acc_now, jerk_now = self._trajectory.get_state(t)

    # Start waypoint velocities
    vel[0:6, 0] = vel_now

    # End waypoint velocities
    vel[0, 1] = self._dir_vel
    vel[1, 1] = self._yaw_vel
    vel[2, 1] = knee_velocity
    vel[3:6, 1] = arm_velocity

    # Start waypoint accel
    acc[0:6, 0] = acc_now

    # Start waypoint jerk
    jrk[0:6, 0] = jerk_now

    # End waypoint accel and jerk are always zero
    self._trajectory = self._create_trajectory()

  def integrate_step(self, dt):
    """
    Called by Igor. User should not call this directly.
    :param dt: integral timestep
    """
    self._hip_pitch = self._hip_pitch+self._hip_pitch_vel*dt

  def update_velocity_controller(self, dt, velocities, wheel_radius, height_com, lean_angle_vel, robot_mass, fbk_lean_angle):
    velP = 15.0 # 5 / .33
    velI = 0.1  # 3 / 30
    velD = .3   # .3 / 1

    inv_dt = 1/dt
    l_wheel_vel = velocities[0]
    r_wheel_vel = velocities[1]

    fbk_chassis_vel = wheel_radius*(l_wheel_vel-r_wheel_vel)*0.5+(height_com*lean_angle_vel)
    cmd_chassis_vel = self._velocities[0, 0]

    cmd_chassis_accel = (cmd_chassis_vel-self._cmd_chassis_vel_last)*inv_dt
    chassis_accel = (fbk_chassis_vel-self._fbk_chassis_vel_last)*inv_dt
    self._cmd_chassis_vel_last = cmd_chassis_vel
    self._fbk_chassis_vel_last = fbk_chassis_vel

    lean_ff = 0.1*robot_mass*cmd_chassis_accel/height_com
    vel_ff = cmd_chassis_vel/wheel_radius

    self._vel_ff = vel_ff

    vel_error = cmd_chassis_vel-fbk_chassis_vel
    self._vel_error = vel_error
    vel_error_cum = self._vel_error_cum+(vel_error*dt)
    self._vel_error_cum = np.clip(vel_error_cum, -50.0, 50.0)

    cmd_lean_angle = (velP*vel_error)+(velI*vel_error_cum)+(velD*chassis_accel)+lean_ff
    lean_angle_error = fbk_lean_angle-cmd_lean_angle
    lean_angle_error_cum = self._lean_angle_error_cum+(lean_angle_error*dt)
    self._lean_angle_error_cum = np.clip(lean_angle_error_cum, -0.2, 0.2)

  @property
  def vel_ff(self):
    return self._vel_ff

  @property
  def lean_angle_error(self):
    return self._lean_angle_error

  @property
  def lean_angle_error_cum(self):
    return self._lean_angle_error_cum

  @property
  def hip_pitch(self):
    return self._hip_pitch

  @property
  def directional_velocity(self):
    return self._velocities[0, 0]

  @property
  def yaw_velocity(self):
    return self._velocities[1, 0]

  @property
  def knee_velocity(self):
    return self._velocities[2, 0]

  @property
  def hip_velocity(self):
    return self._velocities[2, 0]*0.5

  def set_directional_velocity(self, velocity):
    """
    TODO: document
    :param velocity:
    :return:
    """
    self.acquire_value_lock()
    self._dir_vel = velocity
    self.release_value_lock()

  def set_yaw_velocity(self, velocity):
    """
    TODO: document
    :param velocity:
    :return:
    """
    self.acquire_value_lock()
    self._yaw_vel = velocity
    self.release_value_lock()


class Arm(PeripheralBody):

  damper_gains = np.matrix([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64).T
  spring_gains = np.matrix([100.0, 10.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64).T

  def __init__(self, val_lock, name, group_indices):
    assert name == 'Left' or name == 'Right'
    super(Arm, self).__init__(val_lock, group_indices, name)

    self._name = name
    self._vel = np.zeros(3, dtype=np.float64)
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
    self._wrist_vel = 0.0

    # Additionally, calculate determinant of jacobians
    self._current_det_actual = None
    self._current_det_expected = None

  def integrate_step(self, dt, positions):
    """
    Called by Igor. User should not call this directly.
    :param dt:
    """
    vel = self._vel
    vel_dt = vel*dt
    it = self._grip_pos + vel_dt
    self._new_grip_pos[:] = it
    robot = self._robot
    positions = positions[self.group_indices]
    xyz_objective = hebi.robot_model.endeffector_position_objective(self._new_grip_pos)
    new_arm_joint_angs = robot.solve_inverse_kinematics(positions, xyz_objective)

    #new_arm_joint_vels = np.linalg.lstsq(self.current_j_actual[0:3, 0:3], self._vel, rcond=None)[0]

    jacobian_new = robot.get_jacobian_end_effector(new_arm_joint_angs)[0:3, 0:3]
    det_J_new = abs(np.linalg.det(jacobian_new))

    if (self._current_det_expected < 0.010) and (det_J_new < self._current_det_expected):
      # Near singularity - don't command arm towards it
      self._joint_velocities[0:3] = 0.0
    else:
      self._joint_velocities[0:3] = np.linalg.solve(self.current_j_actual[0:3, 0:3], self._vel)
      self._joint_angles[0:3] = new_arm_joint_angs[0:3]
      self._grip_pos[:] = self._new_grip_pos

    wrist_vel_cmd = self._wrist_vel

    self._joint_velocities[3] = self._joint_velocities[1]+self._joint_velocities[2]+(self._direction*wrist_vel_cmd)
    self._joint_angles[3] = self._joint_angles[3] + self._joint_velocities[3]*dt

  @property
  def current_det_actual(self):
    return self._current_det_actual

  @property
  def current_det_expected(self):
    return self._current_det_expected

  @property
  def velocity(self):
    # AKA `grip_vel`
    return self._vel

  @property
  def wrist_velocity(self):
    return self._wrist_vel

  @property
  def grip_position(self):
    return self._grip_pos

  def update_position(self, positions, commanded_positions):
    super(Arm, self).update_position(positions, commanded_positions)
    self._current_det_actual = np.linalg.det(self._current_j_actual[0:4, 0:4])
    self._current_det_expected = np.linalg.det(self._current_j_expected[0:4, 0:4])

  def update_command(self, group_command, positions, velocities, pose, soft_start):
    pos = self._joint_angles
    vel = self._joint_velocities

    positions = positions[self.group_indices]
    velocities = velocities[self.group_indices]

    # Positions and velocities are already known at this point - don't need to calculate them

    # ----------------
    # Calculate effort
    xyz_error = self._grip_pos-self.current_tip_fk[0:3, 3].ravel()
    pos_error = np.zeros((6, 1), dtype=np.float64)
    pos_error[0:3] = xyz_error.T
    vel_error = np.matrix(vel - velocities).T
    vel_error = self.current_j_actual * vel_error

    pos_dot = np.multiply(Arm.spring_gains, pos_error)
    vel_dot = np.multiply(Arm.damper_gains, vel_error)
    impedance_torque = self.current_j_actual.T*(pos_dot+vel_dot)
    grav_comp_torque = math_func.get_grav_comp_efforts(self._robot, positions, -pose[2, 0:3])

    effort = soft_start*impedance_torque.ravel()+grav_comp_torque.ravel()

    # Send commands
    idx = 0
    for i in self.group_indices:
      cmd = group_command[i]
      cmd.position = pos[idx]
      cmd.velocity = vel[idx]
      cmd.effort = effort[0, idx]
      idx = idx + 1

  def set_x_velocity(self, value):
    self.acquire_value_lock()
    self._vel[0] = value
    self.release_value_lock()

  def set_y_velocity(self, value):
    self.acquire_value_lock()
    self._vel[1] = value
    self.release_value_lock()

  def set_z_velocity(self, value):
    self.acquire_value_lock()
    self._vel[2] = value
    self.release_value_lock()

  def set_wrist_velocity(self, value):
    self.acquire_value_lock()
    self._wrist_vel = value
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
    self.__hip_t = hip_t

    kin = self._robot
    kin.add_actuator('X5-9')
    kin.add_link('X5', extension=0.375, twist=np.pi)
    kin.add_actuator('X5-4')
    kin.add_link('X5', extension=0.325, twist=np.pi)

    home_knee_angle = np.deg2rad(130)
    home_hip_angle = (np.pi*0.5)+(home_knee_angle*0.5)

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

    self._hip_angle = 0.0
    self._knee_angle = 0.0
    self._knee_velocity = 0.0
    self._knee_angle_max = 2.65
    self._knee_angle_min = 0.65

    # Additionally calculate commanded end effector position
    self._current_cmd_tip_fk = None

  def integrate_step(self, dt, hip_pitch):
    """
    Called by Igor. User should not call this directly.

    :param dt:       integral timestep
    :param hip_pitch:
    """
    self._knee_angle = self._knee_angle+self._knee_velocity*dt
    self._hip_angle = (np.pi+self._knee_angle)*0.5+hip_pitch

  def update_position(self, positions, commanded_positions):
    super(Leg, self).update_position(positions, commanded_positions)
    self._current_cmd_tip_fk = self._robot.get_forward_kinematics('endeffector', commanded_positions[self.group_indices])[0]

  def update_command(self, group_command, commanded_velocities, velocities, roll_angle, soft_start):
    indices = self.group_indices
    hip_idx = indices[0]
    knee_idx = indices[1]

    hip_cmd = group_command[hip_idx]
    knee_cmd = group_command[knee_idx]

    velocities = velocities[self.group_indices]
    commanded_velocities = commanded_velocities[self.group_indices]
    vel_error = np.matrix(commanded_velocities-velocities).T

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
    impedance_torque = self.current_j_actual.T*(pos_dot + vel_dot + e_term)
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
  def knee_velocity(self):
    return self._knee_velocity

  def set_knee_velocity(self, vel):
    self.acquire_value_lock()
    self._knee_velocity = vel
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


import wx
import wx.grid as gridlib
from pubsub import pub

class Form(wx.Frame):

  def __init__(self, igor):
    self._igor = igor
    self._group_command = hebi.GroupCommand(igor.group.size)

    wx.Frame.__init__(self, parent=None, title="Igor II Data")
    panel = wx.Panel(self)

    #font = wx.SystemSettings_GetFont(wx.SYS_SYSTEM_FONT)
    #font.SetPointSize(9)

    vbox = wx.BoxSizer(wx.VERTICAL)
    hbox_l_leg = wx.BoxSizer(wx.HORIZONTAL)
    hbox_r_leg = wx.BoxSizer(wx.HORIZONTAL)
    hbox_l_arm = wx.BoxSizer(wx.HORIZONTAL)
    hbox_r_arm = wx.BoxSizer(wx.HORIZONTAL)
    hbox_wheel = wx.BoxSizer(wx.HORIZONTAL)

    # Labels
    l_leg_st = wx.StaticText(panel, label='Left Leg')
    r_leg_st = wx.StaticText(panel, label='Right Leg')
    l_arm_st = wx.StaticText(panel, label='Left Arm')
    r_arm_st = wx.StaticText(panel, label='Right Arm')
    wheel_st = wx.StaticText(panel, label='Wheels')
    #l_leg_st.SetFont(font)
    #r_leg_st.SetFont(font)
    #l_arm_st.SetFont(font)
    #r_arm_st.SetFont(font)

    # Grids
    l_leg_grid = gridlib.Grid(panel)
    r_leg_grid = gridlib.Grid(panel)
    l_arm_grid = gridlib.Grid(panel)
    r_arm_grid = gridlib.Grid(panel)
    wheel_grid = gridlib.Grid(panel)

    # Create size
    l_leg_grid.CreateGrid(3, 2)
    r_leg_grid.CreateGrid(3, 2)
    l_arm_grid.CreateGrid(3, 4)
    r_arm_grid.CreateGrid(3, 4)
    wheel_grid.CreateGrid(1, 2)

    # Set rows for all
    l_leg_grid.SetRowLabelValue(0, "Position")
    r_leg_grid.SetRowLabelValue(0, "Position")
    l_arm_grid.SetRowLabelValue(0, "Position")
    r_arm_grid.SetRowLabelValue(0, "Position")
    wheel_grid.SetRowLabelValue(0, "Velocity")
    l_leg_grid.SetRowLabelValue(1, "Velocity")
    r_leg_grid.SetRowLabelValue(1, "Velocity")
    l_arm_grid.SetRowLabelValue(1, "Velocity")
    r_arm_grid.SetRowLabelValue(1, "Velocity")
    l_leg_grid.SetRowLabelValue(2, "Effort")
    r_leg_grid.SetRowLabelValue(2, "Effort")
    l_arm_grid.SetRowLabelValue(2, "Effort")
    r_arm_grid.SetRowLabelValue(2, "Effort")

    # Set leg cols
    l_leg_grid.SetColLabelValue(0, "Hip")
    l_leg_grid.SetColLabelValue(1, "Knee")
    r_leg_grid.SetColLabelValue(0, "Hip")
    r_leg_grid.SetColLabelValue(1, "Knee")

    # Set arm cols
    l_arm_grid.SetColLabelValue(0, "Base")
    r_arm_grid.SetColLabelValue(0, "Base")
    l_arm_grid.SetColLabelValue(1, "Shoulder")
    r_arm_grid.SetColLabelValue(1, "Shoulder")
    l_arm_grid.SetColLabelValue(2, "Elbow")
    r_arm_grid.SetColLabelValue(2, "Elbow")
    l_arm_grid.SetColLabelValue(3, "Wrist")
    r_arm_grid.SetColLabelValue(3, "Wrist")

    # Wheel cols
    wheel_grid.SetColLabelValue(0, "Left")
    wheel_grid.SetColLabelValue(1, "Right")

    self._l_leg_grid = l_leg_grid
    self._r_leg_grid = r_leg_grid
    self._l_arm_grid = l_arm_grid
    self._r_arm_grid = r_arm_grid
    self._wheel_grid = wheel_grid

    # Data
    self._l_leg_pos = np.empty(2, dtype=float)
    self._l_leg_vel = np.empty(2, dtype=float)
    self._l_leg_eff = np.empty(2, dtype=float)
    self._r_leg_pos = np.empty(2, dtype=float)
    self._r_leg_vel = np.empty(2, dtype=float)
    self._r_leg_eff = np.empty(2, dtype=float)
    self._l_arm_pos = np.empty(4, dtype=float)
    self._l_arm_vel = np.empty(4, dtype=float)
    self._l_arm_eff = np.empty(4, dtype=float)
    self._r_arm_pos = np.empty(4, dtype=float)
    self._r_arm_vel = np.empty(4, dtype=float)
    self._r_arm_eff = np.empty(4, dtype=float)

    self._l_wheel_vel = 0.0
    self._r_wheel_vel = 0.0

    hbox_l_leg.Add(l_leg_st, flag=wx.RIGHT, border=8)
    hbox_l_leg.Add(l_leg_grid, proportion=1)

    hbox_r_leg.Add(r_leg_st, flag=wx.RIGHT, border=8)
    hbox_r_leg.Add(r_leg_grid, proportion=1)

    hbox_l_arm.Add(l_arm_st, flag=wx.RIGHT, border=8)
    hbox_l_arm.Add(l_arm_grid, proportion=1)

    hbox_r_arm.Add(r_arm_st, flag=wx.RIGHT, border=8)
    hbox_r_arm.Add(r_arm_grid, proportion=1)

    hbox_wheel.Add(wheel_st, flag=wx.RIGHT, border=8)
    hbox_wheel.Add(wheel_grid, proportion=1)

    vbox.Add(hbox_l_leg, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox.Add((-1, 10))
    vbox.Add(hbox_r_leg, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox.Add((-1, 10))
    vbox.Add(hbox_l_arm, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox.Add((-1, 10))
    vbox.Add(hbox_r_arm, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox.Add((-1, 10))
    vbox.Add(hbox_wheel, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox.Add((-1, 10))

    vbox.Fit(self)
    panel.SetSizer(vbox)
    pub.subscribe(self._update_container, "update")
    panel.Layout()

  def _update_container(self):
    igor = self._igor
    group_command = self._group_command

    l_leg = igor.left_leg
    r_leg = igor.right_leg
    l_arm = igor.left_arm
    r_arm = igor.right_arm

    l_leg_i = l_leg.group_indices
    r_leg_i = r_leg.group_indices
    l_arm_i = l_arm.group_indices
    r_arm_i = r_arm.group_indices

    pos = group_command.position
    vel = group_command.velocity
    eff = group_command.effort

    self._l_leg_pos[:] = pos[l_leg_i]
    self._l_leg_vel[:] = vel[l_leg_i]
    self._l_leg_eff[:] = eff[l_leg_i]
    self._r_leg_pos[:] = pos[r_leg_i]
    self._r_leg_vel[:] = vel[r_leg_i]
    self._r_leg_eff[:] = eff[r_leg_i]
    self._l_arm_pos[:] = pos[l_arm_i]
    self._l_arm_vel[:] = vel[l_arm_i]
    self._l_arm_eff[:] = eff[l_arm_i]
    self._r_arm_pos[:] = pos[r_arm_i]
    self._r_arm_vel[:] = vel[r_arm_i]
    self._r_arm_eff[:] = eff[r_arm_i]

    self._l_wheel_vel = vel[0]
    self._r_wheel_vel = vel[1]

    for i in range(2):
      self._l_leg_grid.SetCellValue(0, i, str(self._l_leg_pos[i]))
      self._l_leg_grid.SetCellValue(1, i, str(self._l_leg_vel[i]))
      self._l_leg_grid.SetCellValue(2, i, str(self._l_leg_eff[i]))

      self._r_leg_grid.SetCellValue(0, i, str(self._r_leg_pos[i]))
      self._r_leg_grid.SetCellValue(1, i, str(self._r_leg_vel[i]))
      self._r_leg_grid.SetCellValue(2, i, str(self._r_leg_eff[i]))

      self._l_arm_grid.SetCellValue(0, i, str(self._l_arm_pos[i]))
      self._l_arm_grid.SetCellValue(1, i, str(self._l_arm_vel[i]))
      self._l_arm_grid.SetCellValue(2, i, str(self._l_arm_eff[i]))

      self._r_arm_grid.SetCellValue(0, i, str(self._r_arm_pos[i]))
      self._r_arm_grid.SetCellValue(1, i, str(self._r_arm_vel[i]))
      self._r_arm_grid.SetCellValue(2, i, str(self._r_arm_eff[i]))

    for i in range(2, 4):
      self._l_arm_grid.SetCellValue(0, i, str(self._l_arm_pos[i]))
      self._l_arm_grid.SetCellValue(1, i, str(self._l_arm_vel[i]))
      self._l_arm_grid.SetCellValue(2, i, str(self._l_arm_eff[i]))

      self._r_arm_grid.SetCellValue(0, i, str(self._r_arm_pos[i]))
      self._r_arm_grid.SetCellValue(1, i, str(self._r_arm_vel[i]))
      self._r_arm_grid.SetCellValue(2, i, str(self._r_arm_eff[i]))

    self._wheel_grid.SetCellValue(0, 0, str(self._l_wheel_vel))
    self._wheel_grid.SetCellValue(0, 1, str(self._r_wheel_vel))

  def request_update(self):
    self._group_command.position = self._igor._group_command.position
    self._group_command.velocity = self._igor._group_command.velocity
    self._group_command.effort = self._igor._group_command.effort
    wx.CallAfter(pub.sendMessage, "update")


global _gui_frame

def _start_gui(igor):
  app = wx.App()
  frame = Form(igor)
  frame.Show()

  global _gui_frame
  _gui_frame = frame

  app.MainLoop()


# ------------------------------------------------------------------------------
# Igor Class
# ------------------------------------------------------------------------------


class Igor(object):

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
    l_arm = self._left_arm
    r_arm = self._right_arm
    l_leg = self._left_leg
    r_leg = self._right_leg

    # Update gyro values from current modules feedback
    self._pose_gyros[:, :] = gyro.T
    pose_gyros = self._pose_gyros

    imu_frames[0] = l_leg.current_fk[3]
    imu_frames[1] = r_leg.current_fk[3]

    imu_frames[3] = l_leg.current_fk[1]
    imu_frames[5] = r_leg.current_fk[1]

    imu_frames[7] = l_arm.current_fk[1]
    imu_frames[8] = l_arm.current_fk[3]
    imu_frames[9] = l_arm.current_fk[5]

    imu_frames[11] = r_arm.current_fk[1]
    imu_frames[12] = r_arm.current_fk[3]
    imu_frames[13] = r_arm.current_fk[5]

    q_rot = np.empty((3,3), dtype=np.float64)
    rpy = np.empty(3, dtype=np.float64)
    rpy_modules = self._rpy

    for i in range(14):
      rot = imu_frames[i][0:3, 0:3]
      # numpy arrays and matrices are different types, and you can't simply do
      # np_matrix * np_array
      # `np.inner` supports exactly what we are trying to do, though
      pose_gyros[0:3, i] = np.inner(rot, pose_gyros[0:3, i])
      quaternion = orientation[i, 0:4]
      # rotation matrix transpose is same as inverse since `rot` is in SO(3)
      q_rot = math_func.quat2rot(quaternion, q_rot)*rot.T
      math_func.rot2ea(q_rot, rpy)
      rpy_modules[0:3, i] = rpy[0:3]

    # NOTE(rLinks234): Unlike MATLAB, we will not have NaNs here. This should be revisited for coherency though.
    self._pose_gyros_mean = np.mean(rpy_modules[:, self._imu_modules], axis=1)

  def _calculate_lean_angle(self):
    """
    TODO: Finish documenting

    The CoM of all individual bodies and pose estimate have been updated at this point
    """
    rpy_module = self._rpy
    imu_modules = self._imu_modules

    roll_angle = np.mean(rpy_module[0, imu_modules])
    pitch_angle = np.mean(rpy_module[1, imu_modules])
    roll_rot = self._roll_rot
    pitch_rot = self._pitch_rot

    # Update roll (rotation matrix)
    math_func.rotate_x(roll_angle, output=roll_rot)
    # Update pitch (rotation matrix)
    math_func.rotate_y(pitch_angle, output=pitch_rot)

    self._roll_angle = roll_angle
    self._pitch_angle = pitch_angle

    T_pose = self._pose_transform
    T_pose[0:3, 0:3] = pitch_rot * roll_rot

    self._lean_angle = pitch_angle
    self._lean_angle_velocity = self._pose_gyros_mean[1]

    # NOTE(rLinks234): MATLAB `leanR` is `pitch_rot` here

    # Gets the translation vector of the Legs' current end effectors
    legs_ef_xyz = np.concatenate((self._left_leg.current_tip_fk[0:3, 3], self._right_leg.current_tip_fk[0:3, 3])).T
    ground_point = np.mean(legs_ef_xyz, axis=1)
    line_com = np.inner(pitch_rot, self._com-ground_point)
    self._height_com = np.linalg.norm(line_com)
    self._feedback_lean_angle = math.atan2(line_com[0], line_com[2])

    # Update Igor center of mass
    self._com = T_pose[0:3, 0:3]*self._com

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

    while t < 3.0:
      # Limit commands initially
      soft_start = min(t/3.0, 1.0)

      grav_comp_efforts = l_arm.get_grav_comp_efforts(positions, gravity)
      pos, vel, accel = l_arm_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, grav_comp_efforts, l_arm_i)

      grav_comp_efforts = r_arm.get_grav_comp_efforts(positions, gravity)
      pos, vel, accel = r_arm_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, grav_comp_efforts, r_arm_i)

      grav_comp_efforts = l_leg.get_grav_comp_efforts(positions, gravity)
      pos, vel, accel = l_leg_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, grav_comp_efforts, l_leg_i)

      grav_comp_efforts = r_leg.get_grav_comp_efforts(positions, gravity)
      pos, vel, accel = r_leg_t.get_state(t)
      set_command_subgroup_pve(group_command, pos, vel, grav_comp_efforts, r_leg_i)

      # Scale effort
      group_command.effort = soft_start*group_command.effort
      #show_command(group_cmd, t)
      group.send_command(group_command)
      #group.send_feedback_request()
      group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
      positions = group_feedback.position
      t = time()-start_time

  def _spin_once(self, bc):
    """

    :param bc:
    :type bc:  bool
    :return:
    """
    self._group_feedback = self._group.get_next_feedback(reuse_fbk=self._group_feedback)
    group_feedback = self._group_feedback
    group_command = self._group_command

    soft_start = min(time()-self._start_time, 1.0)
    rx_time = group_feedback.receive_time
    #dt = np.mean(rx_time-self._time_last)
    # TEMP: using this for imitation group since the time is always 0 in an imitation group
    dt = max(np.mean(rx_time-self._time_last), 0.01)
    self._time_last[:] = rx_time

    positions = group_feedback.position
    commanded_positions = group_feedback.position_command
    velocities = group_feedback.velocity
    commanded_velocities = group_feedback.velocity_command
    effort = group_feedback.effort
    gyro = group_feedback.gyro
    orientation = group_feedback.orientation

    self._update_com(positions, commanded_positions)
    self._update_pose_estimate(gyro, orientation)
    self._calculate_lean_angle()

    # ----------------------------
    # Value Critical section begin
    self._value_lock.acquire()

    # For now, we only use the left arm velocity values.
    # This is because we send the same values to both the left and right arm,
    # and also because the chassis trajectory only has 3 command entries for arm velocity values

    knee_velocity = self._left_leg.knee_velocity
    arm_velocity = self._left_arm.velocity

    self._chassis.update_trajectory(knee_velocity, arm_velocity)
    self._chassis.integrate_step(dt)
    self._left_leg.integrate_step(dt, self._chassis.hip_pitch)
    self._right_leg.integrate_step(dt, self._chassis.hip_pitch)
    self._left_arm.integrate_step(dt, positions)
    self._right_arm.integrate_step(dt, positions)

    self._chassis.update_velocity_controller(dt, velocities, self._wheel_radius,
                                             self._height_com, self._lean_angle_velocity,
                                             self._mass, self._feedback_lean_angle)

    if bc:
      leanP = 1.0
      leanI = 20.0
      leanD = 10.0

      l_wheel = group_command[0]
      r_wheel = group_command[1]

      p_effort = (leanP*self._chassis.lean_angle_error)+(leanI*self._chassis.lean_angle_error_cum)+(leanD*self._lean_angle_velocity)
      l_wheel.effort = p_effort*soft_start
      r_wheel.effort = -p_effort*soft_start

    # --------------------------------
    # This doesn't seem right to me...
    # Wheel Commands
    max_velocity = 10.0
    l_wheel_vel = min(max(self._chassis.yaw_velocity+self._chassis.vel_ff, -max_velocity), max_velocity)
    r_wheel_vel = min(max(self._chassis.yaw_velocity-self._chassis.vel_ff, -max_velocity), max_velocity)
    group_command[0].velocity = l_wheel_vel
    group_command[1].velocity = r_wheel_vel

    # ------------
    # Leg Commands

    roll_angle = self._roll_angle
    self._left_leg.update_command(group_command, commanded_velocities, velocities, roll_angle, soft_start)
    self._right_leg.update_command(group_command, commanded_velocities, velocities, roll_angle, soft_start)

    # ------------
    # Arm Commands

    self._left_arm.update_command(group_command, positions, velocities, self._pose_transform, soft_start)
    self._right_arm.update_command(group_command, positions, velocities, self._pose_transform, soft_start)

    self._value_lock.release()
    # --------------------------
    # Value Critical section end

    ####
    # Debugging
    global _gui_frame
    if _gui_frame != None:
      _gui_frame.request_update()
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

    This is only called by :meth:`__start`
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
      self._spin_once(bc)
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

    # !-------------------------------------------
    # Weird adjusting of grip position from MATLAB
    grip_pos = np.matrix(np.empty((3, 2), dtype=np.float64))
    for i, val in enumerate(l_arm.grip_position): grip_pos[i, 0] = val
    for i, val in enumerate(r_arm.grip_position): grip_pos[i, 1] = val

    grip_pos[1, 1] = -grip_pos[1, 1]
    grip_pos = matlib.repmat(np.mean(grip_pos, axis=1), 1, 2)
    grip_pos[1, 1] = -grip_pos[1, 1]

    # TODO: grip_pos[0:3, 0] needs to go from (3,1) to (3)
    l_arm._grip_pos[:] = grip_pos[0:3, 0].ravel()
    r_arm._grip_pos[:] = grip_pos[0:3, 1].ravel()

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
    self._lean_angle = 0.0
    self._lean_angle_velocity = 0.0
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
    group.command_lifetime = 100
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
    return self._left_arm

  @property
  def left_leg(self):
    return self._left_leg

  @property
  def right_leg(self):
    return self._left_leg

  @property
  def chassis(self):
    return self._chassis

