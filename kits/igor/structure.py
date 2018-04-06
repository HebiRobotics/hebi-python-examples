import sys, threading

from . import DemoUtils, Joystick
from .config import Igor2Config
from .event_handlers import register_igor_event_handlers

# kits.util._internal.type_utils
from ..util._internal.type_utils import assert_type
import hebi

from time import sleep
from functools import partial as funpart


class Chassis(object):

  def __init__(self):
    pass


class Arm(object):

  def __init__(self, name):
    self.__name = name

  @property
  def name(self):
    return self.__name

  def set_x_velocity(self, value):
    print('{0} Velocity(x): {1}'.format(self.__name, value))

  def set_y_velocity(self, value):
    print('{0} Velocity(y): {1}'.format(self.__name, value))

  def set_z_velocity(self, value):
    print('{0} Velocity(z): {1}'.format(self.__name, value))


class Leg(object):

  def __init__(self, name):
    self.__name = name

  @property
  def name(self):
    return self.__name


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
  def is_main_thread_active():
    return threading.main_thread().is_alive()
else:
  is_main_thread_active = lambda : any((i.name == "MainThread") and i.is_alive() for i in threading.enumerate())

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
    return self.__quit

  def __continue(self):
    """
    Contract: This method assumes the caller has acquired `__state_lock`
    """
    return is_main_thread_active() and not self.__pending_quit()

# ------------------------------------------------
# Lifecycle functions
# ------------------------------------------------

  def __stop(self):
    """
    Stop running Igor. This happens once the user requests the demo to stop,
    or when the running application begins to shut down.

    This is only called by :meth:`__stop`
    """
    # TODO
    print('stopping Igor...')

  def __start(self):
    """
    Main processing method. This runs on a background thread.
    """
    self.__state_lock.acquire()
    while self.__continue():
      # We have `__state_lock` at this point. Access fields here before releasing lock
      bc_enabled = self.__balance_controller_enabled
      self.__state_lock.release()

      # TODO
      sleep(0.5)
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

    # View this function in `event_handlers.py` to see 
    # all of the joystick event handlers registered
    register_igor_event_handlers(self, joy)
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

    self.__joy_dead_zone = 0.06

    self.__chassis = Chassis()
    self.__left_arm = Arm('Left')
    self.__right_arm = Arm('Right')
    self.__left_leg = Leg('Left')
    self.__right_leg = Leg('Right')

    from threading import Lock
    self.__state_lock = Lock()
    self.__started = False
    self.__balance_controller_enabled = True
    self.__quit = False

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
    self.__quit = True
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

