import os

from util.input.joystick import Joystick
from util.input.module_controller import HebiModuleController

# ------------------------------------------------------------------------------
# Controller selectors
# ------------------------------------------------------------------------------


def _joystick_first_available_selector():
  for i in range(Joystick.joystick_count()):
    try:
      return Joystick.at_index(i)
    except:
      pass
  return None


def _joystick_by_index_selector(index):
  try:
    return Joystick.at_index(index)
  except:
    return None


def _joystick_by_name_selector(name):
  for pair in Joystick.available_joysticks():
    try:
      index = pair[0]
      joystick_name = pair[1]
      if name in joystick_name:
        return Joystick.at_index(index)
    except:
      pass
  return None


def _controller_by_mobile_io_selector(family, name, feedback_frequency):
  import hebi
  from time import sleep
  lookup = hebi.Lookup()
  sleep(2)
  group = lookup.get_group_from_names([family], [name])
  if group is None:
    msg = 'Could not find Mobile IO on network with\nfamily: {0}\nname: {1}'.format(family, name)
    print(msg)
    raise RuntimeError(msg)
  group.feedback_frequency = feedback_frequency
  return HebiModuleController(group)


def _create_controller_selector(strategy, arg=None):
  if strategy == 'first':
    return _joystick_first_available_selector
  elif strategy == 'index':
    assert arg is not None
    return lambda: _joystick_by_index_selector(arg)
  elif strategy == 'name':
    assert arg is not None
    return lambda: _joystick_by_name_selector(arg)
  elif strategy == 'mobile_io':
    assert arg is not None
    return lambda: _controller_by_mobile_io_selector(arg[0], arg[1], arg[2])
  else:
    raise RuntimeError('Invalid controller selector strategy {0}'.format(strategy))


# ------------------------------------------------------------------------------
# Controller Mappings
# ------------------------------------------------------------------------------


class IgorControllerMapping(object):
  __slots__ = ('_arm_vel_x', '_arm_vel_y', '_stance_height', '_wrist_vel',
    '_chassis_yaw', '_chassis_vel', '_exit_idle_modle', '_quit', '_balance_controller_toggle',
    '_soft_shutdown', '_lower_arm', '_raise_arm', '_stance_height_control_strategy', 
    '_wrist_velocity_control_strategy')

  def __init__(self, arm_vel_x, arm_vel_y, stance_height, wrist_vel, chassis_yaw,
    chassis_vel, exit_idle_modle, quit, balance_controller_toggle, soft_shutdown,
    lower_arm, raise_arm, stance_height_control_strategy, wrist_velocity_control_strategy):

    self._arm_vel_x = arm_vel_x
    self._arm_vel_y = arm_vel_y
    self._stance_height = stance_height
    self._wrist_vel = wrist_vel
    self._chassis_yaw = chassis_yaw
    self._chassis_vel = chassis_vel
    self._exit_idle_modle = exit_idle_modle
    self._quit = quit
    self._balance_controller_toggle = balance_controller_toggle
    self._soft_shutdown = soft_shutdown
    self._lower_arm = lower_arm
    self._raise_arm = raise_arm
    if stance_height_control_strategy == 'SLIDER':
      if type(stance_height) is not str:
        raise TypeError('stance_height must be a string for `SLIDER` stance height control strategy')
    elif stance_height_control_strategy == 'TRIGGERS':
      if type(stance_height) is not tuple:
        raise TypeError('stance_height must be a tuple for `TRIGGERS` stance height control strategy')
    else:
      raise ValueError('Invalid stance height control strategy {0}'.format(stance_height_control_strategy))

    if wrist_velocity_control_strategy == 'SLIDER':
      if type(wrist_vel) is not str:
        raise TypeError('wrist_vel must be a string for `SLIDER` wrist velocity control strategy')
    elif wrist_velocity_control_strategy == 'BUTTONS':
      if type(wrist_vel) is not tuple:
        raise TypeError('wrist_vel must be a tuple for `BUTTONS` wrist velocity control strategy')
    else:
      raise ValueError('Invalid wrist velocity control strategy {0}'.format(wrist_velocity_control_strategy))
  
    self._wrist_velocity_control_strategy = wrist_velocity_control_strategy
    self._stance_height_control_strategy = stance_height_control_strategy

  @property
  def arm_vel_x(self):
    return self._arm_vel_x

  @property
  def arm_vel_y(self):
    return self._arm_vel_y

  @property
  def stance_height(self):
    return self._stance_height

  @property
  def wrist_vel(self):
    return self._wrist_vel

  @property
  def chassis_yaw(self):
    return self._chassis_yaw

  @property
  def chassis_vel(self):
    return self._chassis_vel

  @property
  def exit_idle_modle(self):
    return self._exit_idle_modle

  @property
  def quit(self):
    return self._quit

  @property
  def balance_controller_toggle(self):
    return self._balance_controller_toggle

  @property
  def soft_shutdown(self):
    return self._soft_shutdown

  @property
  def lower_arm(self):
    return self._lower_arm

  @property
  def raise_arm(self):
    return self._raise_arm

  @property
  def stance_height_control_strategy(self):
    return self._stance_height_control_strategy

  @property
  def wrist_velocity_control_strategy(self):
    return self._wrist_velocity_control_strategy


_default_joystick_mapping = IgorControllerMapping('LEFT_STICK_Y', 'LEFT_STICK_X', ('L2', 'R2'), ('DPAD_DOWN', 'DPAD_UP'), 'RIGHT_STICK_X', 'RIGHT_STICK_Y', 'L3', 'SHARE', 'TOUCHPAD', 'OPTIONS', 'L1', 'R1', 'TRIGGERS', 'BUTTONS')
_default_mobile_io_mapping = IgorControllerMapping('a2', 'a1', 'a3', 'a6', 'a7', 'a8', 'b3', 'b1', 'b2', 'b4', 'b8', 'b6', 'SLIDER', 'SLIDER')


# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------


class Igor2Config(object):
  """
  Used when starting up Igor II.

  By default, the configuration selects the first available joystick.
  To select a different joystick selection strategy, you must call
  one of the select_joystick_by_* methods.
  """

  def __init__(self, imitation=False):
    self.__module_names = ['wheel1', 'wheel2',
                           'hip1', 'knee1',
                           'hip2', 'knee2',
                           'base1', 'shoulder1', 'elbow1', 'wrist1',
                           'base2', 'shoulder2', 'elbow2', 'wrist2',
                           'camTilt']
    self.__family = 'Igor II'
    resource_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resources'))
    self.__default_gains = os.path.join(resource_path, 'igorGains.xml')
    self.__default_gains_no_cam = os.path.join(resource_path, 'igorGains_noCamera.xml')
    self.__imitation = imitation
    self.__find_joystick_strategy = None
    self.__controller_mapping = None
    self.select_first_available_joystick()

  def select_joystick_by_name(self, name):
    """
    Set the joystick selection strategy to select a joystick by name
    """
    self.__find_joystick_strategy = _create_controller_selector('name', name)
    self.__controller_mapping = _default_joystick_mapping

  def select_joystick_by_index(self, index):
    """
    Set the joystick selection strategy to select a joystick by index
    """
    self.__find_joystick_strategy = _create_controller_selector('index', index)
    self.__controller_mapping = _default_joystick_mapping

  def select_first_available_joystick(self):
    """
    Set the joystick selection strategy to select the first available joystick
    """
    self.__find_joystick_strategy = _create_controller_selector('first')
    self.__controller_mapping = _default_joystick_mapping

  def select_controller_by_mobile_io(self, family, name, feedback_frequency=200):
    """
    Set the joystick selection strategy to select a mobile IO module with the provided family and name
    """
    self.__find_joystick_strategy = _create_controller_selector('mobile_io', (family, name, feedback_frequency))
    self.__controller_mapping = _default_mobile_io_mapping

  @property
  def module_names(self):
    return self.__module_names

  @property
  def module_names_no_cam(self):
    return self.__module_names[0:-1]

  @property
  def family(self):
    return self.__family

  @property
  def gains_xml(self):
    return self.__default_gains

  @property
  def gains_no_camera_xml(self):
    return self.__default_gains_no_cam

  @property
  def is_imitation(self):
    return self.__imitation

  @property
  def joystick_selector(self):
    return self.__find_joystick_strategy

  @property
  def controller_mapping(self):
    return self.__controller_mapping
  
