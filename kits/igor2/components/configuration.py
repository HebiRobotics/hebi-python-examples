import os

from util.input.joystick import Joystick
from util.input.module_controller import HebiModuleController

# ------------------------------------------------------------------------------
# Joystick selectors
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


def _joystick_by_mobile_io_selector(family, name, feedback_frequency):
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


def _create_joystick_selector(strategy, arg=None):
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
    return lambda: _joystick_by_mobile_io_selector(arg[0], arg[1], arg[2])
  else:
    raise RuntimeError('Invalid strategy {0}'.format(strategy))


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
    self.__find_joystick_strategy = _create_joystick_selector('name', name)

  def select_joystick_by_index(self, index):
    """
    Set the joystick selection strategy to select a joystick by index
    """
    self.__find_joystick_strategy = _create_joystick_selector('index', index)

  def select_first_available_joystick(self):
    """
    Set the joystick selection strategy to select the first available joystick
    """
    self.__find_joystick_strategy = _create_joystick_selector('first')

  def select_joystick_by_mobile_io(self, family, name, feedback_frequency=200):
    """
    Set the joystick selection strategy to select a mobile IO module with the provided family and name
    """
    self.__find_joystick_strategy = _create_joystick_selector('mobile_io', (family, name, feedback_frequency))

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
