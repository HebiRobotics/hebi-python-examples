import sys, threading
import numpy as np

from . import DemoUtils, Joystick
from .config import Igor2Config
from .event_handlers import register_igor_event_handlers

# kits.util._internal.type_utils
from ..util._internal.type_utils import assert_instance, assert_type
from ..util import math_func
import hebi

from time import sleep, time
from functools import partial as funpart


class Chassis(object):

  def __init__(self):
    self.__com = np.array([0.0, 0.0, 0.10+0.3], dtype=np.float64)
    self.__mass = 6.0
    pass


  @property
  def com(self):
    return self.__com

  @property
  def mass(self):
    return self.__mass


class Arm(object):

  def __init__(self, name, group_indices):
    assert name == 'Left' or name == 'Right'

    self.__name = name
    self.__group_indices = group_indices
    base_frame = np.identity(4, dtype=np.float64)
    kin = hebi.robot_model.RobotModel()
    kin.add_actuator('X5-4')

    if name == 'Left':
      mounting = 'left-inside'
      base_frame[0:3, 3] = [0.0, 0.10, 0.20]
      home_angles = np.array([0.0, 20.0, 60.0, 0.0], dtype=np.float64)
    else:
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
    self.__kin = kin

    home_angles = np.deg2rad(home_angles)
    self.__home_angles = home_angles
    self.__home_ef = kin.get_forward_kinematics('endeffector', home_angles)

  @property
  def group_indices(self):
    """
    :return: a list of integers corresponding to the modules
    this Arm represents in the Igor group
    :rtype:  list
    """
    return self.__group_indices

  @property
  def name(self):
    return self.__name

  def create_home_trajectory(self, fbk, duration=3.0):
    """
    Create a trajectory from the current pose to the home pose.
    This is used on soft startup
    ^^^ Is this the correct usage of "pose" ???
    :return:
    """
    assert_type(duration, float, 'duration')
    if duration < 1.0:
      raise ValueError('duration must be greater than 1.0 second')

    num_joints = len(self.__group_indices)
    num_waypoints = 2
    dim = (num_joints, num_waypoints)

    current_positions = fbk.position[self.__group_indices]
    home_angles = self.__home_angles

    times = np.array([0.0, duration], dtype=np.float64)
    pos = np.empty(dim, dtype=np.float64)
    vel = np.zeros(dim, dtype=np.float64)
    accel = np.zeros(dim, dtype=np.float64)

    pos[:, 0] = current_positions
    pos[:, 1] = home_angles

    return hebi.trajectory.create_trajectory(times, pos, vel, accel)

  def set_x_velocity(self, value):
    print('{0} Velocity(x): {1}'.format(self.__name, value))

  def set_y_velocity(self, value):
    print('{0} Velocity(y): {1}'.format(self.__name, value))

  def set_z_velocity(self, value):
    print('{0} Velocity(z): {1}'.format(self.__name, value))


class Leg(object):
  """
  Represents a leg (and wheel)
  """

  def __init__(self, name, group_indices):
    assert name == 'Left' or name == 'Right'

    self.__name = name
    self.__group_indices = group_indices
    base_frame = np.identity(4, dtype=np.float64)

    hip_t = np.identity(4, dtype=np.float64)
    hip_t[0:3, 0:3] = math_func.rotate_x(np.pi*0.5)
    hip_t[0:3, 3] = [0.0, 0.0225, 0.055]
    self.__hip_t = hip_t

    kin = hebi.robot_model.RobotModel()
    kin.add_actuator('X5-9')
    kin.add_link('X5', extension=0.375, twist=np.pi)
    kin.add_actuator('X5-4')
    kin.add_link('X5', extension=0.325, twist=np.pi)

    home_knee_angle = np.deg2rad(130)
    home_hip_angle = np.pi*0.5+home_knee_angle*0.5

    if name == 'Left':
      home_angles = np.array([home_hip_angle, home_knee_angle], dtype=np.float64)
      base_frame[0:3, 3] = [0.0, 0.15, 0.0]
      base_frame[0:3, 0:3] = math_func.rotate_x(np.pi * -0.5)
    else:
      home_angles = np.array([-home_hip_angle, -home_knee_angle], dtype=np.float64)
      base_frame[0:3, 3] = [0.0, -0.15, 0.0]
      base_frame[0:3, 0:3] = math_func.rotate_x(np.pi*0.5)

    kin.base_frame = base_frame
    self.__kin = kin
    self.__home_angles = home_angles
    self.__wheel_radius = 0.100
    self.__wheel_base = 0.43

  @property
  def group_indices(self):
    """
    :return: a list of integers corresponding to the modules
    this Leg represents in the Igor group
    :rtype:  list
    """
    return self.__group_indices

  @property
  def name(self):
    return self.__name

  @property
  def wheel_radius(self):
    return self.__wheel_radius

  @property
  def wheel_base(self):
    return self.__wheel_base

  @property
  def knee_angle(self):
    """
    :return: the commanded knee angle in radians
    :rtype:  float
    """

  @property
  def hip_angle(self):
    """
    :return: the commanded hip angle in radians
    :rtype:  float
    """

  def create_home_trajectory(self, fbk, duration=3.0):
    """
    Create a trajectory from the current pose to the home pose.
    This is used on soft startup
    ^^^ Is this the correct usage of "pose" ???
    :return:
    """
    assert_type(duration, float, 'duration')
    if duration < 1.0:
      raise ValueError('duration must be greater than 1.0 second')
    num_joints = len(self.__group_indices)
    num_waypoints = 2
    dim = (num_joints, num_waypoints)

    current_positions = fbk.position[self.__group_indices]
    home_angles = self.__home_angles

    times = np.array([0.0, duration], dtype=np.float64)
    pos = np.empty(dim, dtype=np.float64)
    vel = np.zeros(dim, dtype=np.float64)
    accel = np.zeros(dim, dtype=np.float64)

    pos[:, 0] = current_positions
    pos[:, 1] = home_angles

    return hebi.trajectory.create_trajectory(times, pos, vel, accel)


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
# Igor Class
# ------------------------------------------------------------------------------


class Igor(object):

# ------------------------------------------------
# Helper functions
# ------------------------------------------------

  def __ensure_started(self):
    """
    Contract: This method assumes the caller has acquired `__state_lock`
    """
    if not self.__started:
      self.__state_lock.release()
      raise RuntimeError('Igor has not been started (igor.start() was not called)')

  def __pending_quit(self):
    """
    Contract: This method assumes the caller has acquired `__state_lock`
    """
    return self.__quit_flag

  def __continue(self):
    """
    Contract: This method assumes the caller has acquired `__state_lock`
    """
    return is_main_thread_active() and not self.__pending_quit()

# ------------------------------------------------
# Actions
# ------------------------------------------------

  def __soft_startup(self):
    l_arm = self.__left_arm
    r_arm = self.__right_arm
    l_leg = self.__left_leg
    r_leg = self.__right_leg

    l_arm_i = l_arm.group_indices
    r_arm_i = r_arm.group_indices
    l_leg_i = l_leg.group_indices
    r_leg_i = r_leg.group_indices

    group = self.__group
    fbk = hebi.GroupFeedback(group.size)
    cmd = hebi.GroupCommand(group.size)

    group.send_feedback_request()
    fbk = group.get_next_feedback(reuse_fbk=fbk)

    l_arm_t = l_arm.create_home_trajectory(fbk)
    r_arm_t = r_arm.create_home_trajectory(fbk)
    l_leg_t = l_leg.create_home_trajectory(fbk)
    r_leg_t = r_leg.create_home_trajectory(fbk)

    start_time = time()
    t = 0.0

    while t < 3.0:
      # Limit commands initially
      soft_start = min(t/3.0, 1.0)

      pos, vel, accel = l_arm_t.get_state(t)
      idx = 0
      for i in l_arm_i:
        cmd[i].position = pos[idx]
        cmd[i].velocity = vel[idx]
        # TODO: gravComp for effort

      pos, vel, accel = r_arm_t.get_state(t)
      idx = 0
      for i in r_arm_i:
        cmd[i].position = pos[idx]
        cmd[i].velocity = vel[idx]
        # TODO: gravComp for effort

      pos, vel, accel = l_leg_t.get_state(t)
      idx = 0
      for i in l_leg_i:
        cmd[i].position = pos[idx]
        cmd[i].velocity = vel[idx]
        # TODO: gravComp for effort

      pos, vel, accel = r_leg_t.get_state(t)
      idx = 0
      for i in r_leg_i:
        cmd[i].position = pos[idx]
        cmd[i].velocity = vel[idx]
        # TODO: gravComp for effort

      group.send_command(cmd)
      fbk = group.get_next_feedback(reuse_fbk=fbk)
      t = time() - start_time


  def __spin_once(self, bc):
    """

    :param bc:
    :type bc:  bool
    :return:
    """
  #TODO: Main loop
  sleep(0.5)

# ------------------------------------------------
# Lifecycle functions
# ------------------------------------------------

  def __stop(self):
    """
    Stop running Igor. This happens once the user requests the demo to stop,
    or when the running application begins to shut down.

    This is only called by :meth:`__start`
    """
    # TODO
    print('stopping Igor...')

  def __start(self):
    """
    Main processing method. This runs on a background thread.
    """
    self.__soft_startup()

    # Delay registering event handlers until now, so Igor
    # can start up without being interrupted by user commands.
    # View this function in `event_handlers.py` to see
    # all of the joystick event handlers registered
    register_igor_event_handlers(self, self.__joy)

    self.__state_lock.acquire()
    while self.__continue():
      # We have `__state_lock` at this point. Access fields here before releasing lock
      bc = self.__balance_controller_enabled
      self.__state_lock.release()
      self.__spin_once(bc)
      self.__state_lock.acquire()

    self.__stop()

# ------------------------------------------------
# Initialization functions
# ------------------------------------------------

  def __find_joystick(self):
    group = self.__group
    group_command = hebi.GroupCommand(group.size)

    if self.__config.is_imitation:
      joy = DemoUtils.retry_on_error(get_first_joystick)
    else:
      on_error = funpart(on_error_find_joystick, group, group_command)
      joy = DemoUtils.retry_on_error(get_first_joystick, on_error)

    self.__joy = joy

  def __load_gains(self):
    group = self.__group
    group.feedback_frequency = 100.0

    # Bail out if group is imitation
    if self.__config.is_imitation:
      return

    gains_command = hebi.GroupCommand(group.size)
    sleep(0.1)

    if self.__has_camera:
      gains_command.load_gains(self.__config.gains_xml)
    else:
      gains_command.load_gains(self.__config.gains_no_camera_xml)
    
    # Send gains multiple times
    for i in range(3):
      group.send_command(gains_command)
      sleep(0.1)

  def __init__(self, has_camera=True, config=None):
    if config == None:
      self.__config = Igor2Config()
    else:
      assert_type(config, Igor2Config, 'config')
      self.__config = config

    self.__has_camera = has_camera
    self.__joy = None
    self.__group = None
    self.__proc_thread = None

    self.__joy_dead_zone = 0.06

    self.__chassis = Chassis()
    self.__left_arm = Arm('Left', [6, 7, 8, 9])
    self.__right_arm = Arm('Right', [10, 11, 12, 13])
    self.__left_leg = Leg('Left', [2, 3])
    self.__right_leg = Leg('Right', [4, 5])

    from threading import Lock
    self.__state_lock = Lock()
    self.__started = False
    self.__balance_controller_enabled = True
    self.__quit_flag = False

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
    self.__state_lock.acquire()
    if self.__started:
      self.__state_lock.release()
      return
    
    self.__group = create_group(self.__config, self.__has_camera)
    self.__find_joystick()
    self.__load_gains()

    from threading import Condition, Lock, Thread
    start_condition = Condition(Lock())

    def start_routine(start_condition):
      start_condition.acquire()

      # Calling thread holds `__state_lock` AND
      # is also blocked until we call `start_condition.notify_all()`
      # So we can safely modify this here.
      self.__started = True
      
      # Allow the calling thread to continue
      start_condition.notify_all()
      start_condition.release()

      self.__start()

    self.__proc_thread = Thread(target=start_routine, name='Igor II Controller',
                                args = (start_condition,))
    
    # We will have started the thread before returning,
    # but make sure the function has begun running before
    # we release the `__state_lock` and return
    start_condition.acquire()
    self.__proc_thread.start()
    start_condition.wait()
    start_condition.release()

    self.__state_lock.release()

  def request_stop(self):
    """
    Send a request to stop the demo.
    If the demo has not been started, this method raises an exception.

    :raises RuntimeError: if `start()` has not been called
    """
    self.__state_lock.acquire()
    self.__ensure_started()
    self.__quit_flag = True
    self.__state_lock.release()

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
    self.__state_lock.acquire()
    self.__ensure_started()
    self.__balance_controller_enabled = state
    self.__state_lock.release()


# ------------------------------------------------
# Properties
# ------------------------------------------------

  @property
  def joystick_dead_zone(self):
    return self.__joy_dead_zone

  @property
  def has_camera(self):
    return self.__has_camera

  @property
  def started(self):
    self.__state_lock.acquire()
    val = self.__started
    self.__state_lock.release()
    return val

  @property
  def left_arm(self):
    return self.__left_arm

  @property
  def right_arm(self):
    return self.__left_arm

  @property
  def left_leg(self):
    return self.__left_leg

  @property
  def right_leg(self):
    return self.__left_leg

  @property
  def chassis(self):
    return self.__chassis

