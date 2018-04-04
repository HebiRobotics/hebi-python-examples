import sys


# ------------------------------------------------------------------------------
# Classes
# ------------------------------------------------------------------------------


class JoystickMappingEntry(object):

  def __call__(self):
    return self.__func()

  def __init__(self, type, index, name, mapper):
    self.__type = type
    self.__index = index
    self.__name = name

    if type == 'Axis':
      self.__func = lambda: mapper.joystick.get_axis(index)
    elif type == 'Button':
      self.__func = lambda: mapper.joystick.get_button(index)
    else:
      raise RuntimeError('Unknown type "{0}"'.format(type))

  @property
  def type(self):
    return self.__type

  @property
  def index(self):
    return self.__index

  @property
  def name(self):
    return self.__name


class JoystickMapper(object):

  def __get_axis(self, name):
    return self.__axes[name]

  def __get_button(self, name):
    return self.__buttons[name]

  def __get_string(self, item):
    if item.startswith('AXIS_'):
      return self.__get_axis(item[5:])
    elif item.startswith('BUTTON_'):
      return self.__get_button(item[7:])
    raise KeyError('Key {0} not found'.format(item))

  def __getitem__(self, item):
    if type(item) == str:
      return self.__get_string(item)
    else:
      raise TypeError('Key must be a str')

  def __init__(self, joystick):
    self.__joy = joystick
    self.__axes = dict()
    self.__buttons = dict()
    self.__axis_ids = set()
    self.__button_ids = set()

  @property
  def axis_ids(self):
    return self.__axis_ids[:]

  @property
  def button_ids(self):
    return self.__button_ids[:]

  @property
  def joystick(self):
    return self.__joy

  def add_axis(self, index, name, key):
    """
    :param index:
    :type index: int
    :param name: human readable representation of the axis
    :type name: str
    :param key: Key of this axis
    :type key: str
    """
    entry = JoystickMappingEntry('Axis', index, name, self)
    self.__axes[key] = entry
    self.__axes[index] = entry
    self.__axis_ids.add('AXIS_' + key.upper())

  def add_button(self, index, name, key):
    """
    :param index:
    :type index: int
    :param name: human readable representation of the button
    :type name: str
    :param key: Key of this button
    :type key: str
    """
    entry = JoystickMappingEntry('Button', index, name, self)
    self.__buttons[key] = entry
    self.__buttons[index] = entry
    self.__button_ids.add('BUTTON_' + key.upper())

  def get_axis(self, axis):
    if type(axis) == int:
      if axis in self.__axes:
        return self.__axes[axis]
      raise IndexError('Axis[{0}] is out of range')
    elif type(axis) == str:
      axiskey = axis.upper()
      if axiskey in self.__axes:
        return self.__axes[axiskey]
      raise KeyError('Axis[{0}] not found'.format(axis))
    raise TypeError(type(axis))

  def get_axis_value(self, axis):
    return self.get_axis(axis)()

  def get_button(self, button):
    if type(button) == int:
      if button in self.__buttons:
        return self.__buttons[button]
      raise IndexError('Button[{0}] is out of range')
    elif type(button) == str:
      buttonkey = button.upper()
      if buttonkey in self.__buttons:
        return self.__buttons[buttonkey]
      raise KeyError('Button[{0}] not found'.format(button))
    raise TypeError(type(button))

  def get_button_value(self, button):
    return self.get_button(button)()



# ------------------------------------------------------------------------------
# Mapping functions
# ------------------------------------------------------------------------------


def __map_ps3_cechzc2u(joystick):
  """
  Proper controller mapping for Sony Playstation 3 Wireless Controller
  Model CECHZC2U
  """
  # TODO: test on OSX and Windows
  d = JoystickMapper(joystick)

  d.add_axis(0, 'Left Stick (x)', 'LEFT_STICK_X')
  d.add_axis(1, 'Left Stick (y)', 'LEFT_STICK_Y')
  d.add_axis(3, 'Right Stick (x)', 'RIGHT_STICK_X')
  d.add_axis(4, 'Right Stick (y)', 'RIGHT_STICK_Y')
  d.add_axis(2, 'Left Trigger', 'LEFT_TRIGGER')
  d.add_axis(5, 'Right Trigger', 'RIGHT_TRIGGER')

  d.add_button(0, 'X', 'X')
  d.add_button(1, 'Circle', 'CIRCLE')
  d.add_button(2, 'Triangle', 'TRIANGLE')
  d.add_button(3, 'Square', 'SQUARE')
  d.add_button(4, 'Left Shoulder', 'LEFT_SHOULDER')
  d.add_button(5, 'Right Shoulder', 'RIGHT_SHOULDER')
  d.add_button(6, 'Left Trigger', 'LEFT_TRIGGER')
  d.add_button(7, 'Right Trigger', 'RIGHT_TRIGGER')
  d.add_button(8, 'Select', 'SELECT')
  d.add_button(9, 'Start', 'START')
  d.add_button(10, 'Home', 'HOME')
  d.add_button(11, 'Left Stick Click', 'LEFT_STICK_CLICK')
  d.add_button(12, 'Right Stick Click', 'RIGHT_STICK_CLICK')
  d.add_button(13, 'DPad Up', 'DPAD_UP')
  d.add_button(14, 'DPad Down', 'DPAD_DOWN')
  d.add_button(15, 'DPad Left', 'DPAD_LEFT')
  d.add_button(16, 'DPad Right', 'DPAD_RIGHT')
  return d


def __map_ps4_cuh_zct2u(joystick):
  """
  Proper controller mapping for Sony Playstation 4 Wireless Controller
  Model CUH-ZXT2U

  """
  d = JoystickMapper(joystick)

  d.add_axis(0, 'Left Stick (x)', 'LEFT_STICK_X')
  d.add_axis(1, 'Left Stick (y)', 'LEFT_STICK_Y')

  d.add_button(9, 'Options', 'OPTIONS')
  d.add_button(4, 'Left Trigger', 'LEFT_TRIGGER')
  d.add_button(5, 'Right Trigger', 'RIGHT_TRIGGER')
  d.add_button(8, 'Share', 'SHARE')

  if (sys.platform == 'darwin' or sys.platform == 'win32'):
    d.add_axis(3, 'Left Trigger', 'LEFT_TRIGGER')
    d.add_axis(4, 'Right Trigger', 'RIGHT_TRIGGER')
    d.add_axis(2, 'Right Stick (x)', 'RIGHT_STICK_X')
    d.add_axis(5, 'Right Stick (y)', 'RIGHT_STICK_Y')

    d.add_button(0, 'Square', 'SQUARE')
    d.add_button(2, 'Circle', 'CIRCLE')
    d.add_button(3, 'Triangle', 'TRIANGLE')
    d.add_button(1, 'X', 'X')
    d.add_button(10, 'Left Stick', 'LEFT_STICK_CLICK')
    d.add_button(13, 'Touchpad', 'TOUCHPAD')
  elif sys.platform.startswith('linux'):
    d.add_axis(2, 'Left Trigger', 'LEFT_TRIGGER')
    d.add_axis(5, 'Right Trigger', 'RIGHT_TRIGGER')
    d.add_axis(3, 'Right Stick (x)', 'RIGHT_STICK_X')
    d.add_axis(4, 'Right Stick (y)', 'RIGHT_STICK_Y')

    d.add_button(3, 'Square', 'SQUARE')
    d.add_button(1, 'Circle', 'CIRCLE')
    d.add_button(2, 'Triangle', 'TRIANGLE')
    d.add_button(0, 'X', 'X')
    d.add_button(11, 'Left Stick', 'LEFT_STICK_CLICK')
    d.add_button(10, 'Touchpad', 'TOUCHPAD')
  else:
    raise RuntimeError('Unknown Operating System/Platform {0}'.format(sys.platform))
  return d


__mappings = {
  '030000004c0500006802000011810000' : __map_ps3_cechzc2u,
  '030000004c050000cc09000011810000' : __map_ps4_cuh_zct2u }


# ------------------------------------------------------------------------------
# Public API
# ------------------------------------------------------------------------------


def get_joystick_mapping(joystick):
  guid = joystick.guid
  if guid in __mappings:
    return __mappings[guid](joystick)
  return None

