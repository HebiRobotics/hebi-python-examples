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

"""
  ARM_VEL_X_AXIS = 'a2'
  ARM_VEL_Y_AXIS = 'a1'
  STANCE_HEIGHT_AXIS = 'a3'
  WRIST_VEL_AXIS = 'a6'
  CHASSIS_YAW_AXIS = 'a7'
  CHASSIS_VEL_AXIS = 'a8'
  QUIT_BTN = 'b1'
  BALANCE_CONTROLLER_TOGGLE_BTN = 'b2'
  SOFT_SHUTDOWN_BTN = 'b4'
  LOWER_ARM_BTN = 'b8'
  RAISE_ARM_BTN = 'b6'
"""


class IgorControllerMapping(object):
  __slots__ = ('_arm_vel_x', '_arm_vel_y', '_stance_height', '_wrist_vel',
    '_chassis_yaw', '_chassis_vel', '_quit', '_balance_controller_toggle',
    '_soft_shutdown', '_lower_arm', '_raise_arm')

  def __init__(self, arm_vel_x, arm_vel_y, stance_height, wrist_vel, chassis_yaw,
    chassis_vel, quit, balance_controller_toggle, soft_shutdown,
    lower_arm, raise_arm):

    self._arm_vel_x = arm_vel_x
    self._arm_vel_y = arm_vel_y
    self._stance_height = stance_height
    self._wrist_vel = wrist_vel
    self._chassis_yaw = chassis_yaw
    self._chassis_vel = chassis_vel
    self._quit = quit
    self._balance_controller_toggle = balance_controller_toggle
    self._soft_shutdown = soft_shutdown
    self._lower_arm = lower_arm
    self._raise_arm = raise_arm

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


_default_joystick_mapping = IgorControllerMapping() # TODO
_default_mobile_io_mapping = IgorControllerMapping('a2', 'a1', 'a3', 'a6', 'a7', 'a8', 'b1', 'b2', 'b4', 'b8', 'b6')


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
    self.select_first_available_joystick()

  def select_joystick_by_name(self, name):
    """
    Set the joystick selection strategy to select a joystick by name
    """
    self.__find_joystick_strategy = _create_controller_selector('name', name)

  def select_joystick_by_index(self, index):
    """
    Set the joystick selection strategy to select a joystick by index
    """
    self.__find_joystick_strategy = _create_controller_selector('index', index)

  def select_first_available_joystick(self):
    """
    Set the joystick selection strategy to select the first available joystick
    """
    self.__find_joystick_strategy = _create_controller_selector('first')

  def select_controller_by_mobile_io(self, family, name, feedback_frequency=200):
    """
    Set the joystick selection strategy to select a mobile IO module with the provided family and name
    """
    self.__find_joystick_strategy = _create_controller_selector('mobile_io', (family, name, feedback_frequency))

  @property
  def module_names(self):
    """
    :return:
    :rtype:  list
    """
    return self.__module_names

  @property
  def module_names_no_cam(self):
    """
    :return:
    :rtype:  list
    """
    return self.__module_names[0:-1]

  @property
  def family(self):
    """
    :return:
    :rtype:  str
    """
    return self.__family

  @property
  def gains_xml(self):
    """
    :return:
    :rtype:  str
    """
    return self.__default_gains

  @property
  def gains_no_camera_xml(self):
    """
    :return:
    :rtype:  str
    """
    return self.__default_gains_no_cam

  @property
  def is_imitation(self):
    """
    :return:
    :rtype:  bool
    """
    return self.__imitation

  @property
  def joystick_selector(self):
    return self.__find_joystick_strategy
