import os, yaml

# ------------------------------------------------------------------------------
# Controller selectors
# ------------------------------------------------------------------------------

def _controller_by_mobile_io_selector(family, name, feedback_frequency):
  import hebi
  from time import sleep
  lookup = hebi.Lookup()
  mio = hebi.util.create_mobile_io(lookup, family, name)
  while mio is None:
    msg = f'Could not find mobileIO on network with\nfamily: {family}\nname: {name}'
    print(msg)
    sleep(1)
    mio = hebi.util.create_mobile_io(lookup, family, name)

  mio._group.feedback_frequency = feedback_frequency
  return mio


# ------------------------------------------------------------------------------
# Controller Mappings
# ------------------------------------------------------------------------------


class IgorControllerMapping(object):
  __slots__ = ('_arm_vel_x', '_arm_vel_y', '_stance_height', '_wrist_vel',
    '_chassis_yaw', '_chassis_vel', '_exit_idle_mode', '_quit', '_balance_controller_toggle',
    '_soft_shutdown', '_lower_arm', '_raise_arm', '_stance_height_control_strategy', 
    '_wrist_velocity_control_strategy', '_i_term_adjust')

  def __init__(self, arm_vel_x, arm_vel_y, stance_height, wrist_vel, chassis_yaw,
    chassis_vel, exit_idle_mode, quit, balance_controller_toggle, soft_shutdown,
    lower_arm, raise_arm, stance_height_control_strategy, wrist_velocity_control_strategy, i_term_adjust=None):

    self._arm_vel_x = arm_vel_x
    self._arm_vel_y = arm_vel_y
    self._stance_height = stance_height
    self._wrist_vel = wrist_vel
    self._chassis_yaw = chassis_yaw
    self._chassis_vel = chassis_vel
    self._exit_idle_mode = exit_idle_mode
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
    self._i_term_adjust = i_term_adjust

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
  def exit_idle_mode(self):
    return self._exit_idle_mode

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

  @property
  def i_term_adjust(self):
    return self._i_term_adjust


_default_mobile_io_mapping = IgorControllerMapping(          'a2',           'a1',         'a3',                     'a6',            'a7',            'a8', 'b3',    'b1',       'b2',      'b4', 'b8', 'b6',   'SLIDER',  'SLIDER', 'a5')


# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------


class Igor2Config(object):
  """
  Used when starting up Igor II.
  """
  def __init__(self, filename='resources/config.yml'):
    with open(filename) as config_file:
        config = yaml.safe_load(config_file)
        self.__module_names = config['names']
    self.__family = 'T-gor'
    resource_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resources'))
    self.__default_gains = os.path.join(resource_path, 'igorGains.xml')
    self.__default_gains_no_cam = os.path.join(resource_path, 'igorGains_noCamera.xml')
    self.__find_joystick_strategy = None
    self.__controller_mapping = None

  def select_controller_by_mobile_io(self, family, name, feedback_frequency=200):
    """
    Set the joystick selection strategy to select a mobile IO module with the provided family and name
    """
    self.__find_joystick_strategy = lambda: _controller_by_mobile_io_selector(family, name, feedback_frequency)
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
  def setup_controller(self):
    return self.__find_joystick_strategy

  @property
  def controller_mapping(self):
    return self.__controller_mapping
  
