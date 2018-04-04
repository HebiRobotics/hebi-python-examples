import sys

class MappingDesc(object):

  def __call__(self):
    return self.__func()

  def __init__(self, type, index, name, joystick):
    self.__type = type
    self.__index = index
    self.__name = name

    if type == 'Axis':
      self.__func = lambda: joystick.get_axis(index)
    elif type == 'Button':
      self.__func = lambda: joystick.get_button(index)
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


_Axis = 'Axis'
_Button = 'Button'


def __map_ps3_cechzc2u(joystick):
  """
  Proper controller mapping for Sony Playstation 3 Wireless Controller
  Model CECHZC2U
  """
  # TODO: test on OSX and Windows
  d = dict()
  d['AXIS_LEFT_STICK_X'] = MappingDesc(_Axis, 0, 'Left Stick (x)', joystick)
  d['AXIS_LEFT_STICK_Y'] = MappingDesc(_Axis, 1, 'Left Stick (y)', joystick)
  d['AXIS_RIGHT_STICK_X'] = MappingDesc(_Axis, 3, 'Right Stick (x)', joystick)
  d['AXIS_RIGHT_STICK_Y'] = MappingDesc(_Axis, 4, 'Right Stick (y)', joystick)
  d['AXIS_LEFT_TRIGGER'] = MappingDesc(_Axis, 2, 'Left Trigger', joystick)
  d['AXIS_RIGHT_TRIGGER'] = MappingDesc(_Axis, 5, 'Right Trigger', joystick)

  d['BUTTON_X'] = MappingDesc(_Button, 0, 'X', joystick)
  d['BUTTON_CIRCLE'] = MappingDesc(_Button, 1, 'Circle', joystick)
  d['BUTTON_TRIANGLE'] = MappingDesc(_Button, 2, 'Triangle', joystick)
  d['BUTTON_SQUARE'] = MappingDesc(_Button, 3, 'Square', joystick)
  d['BUTTON_LEFT_SHOULDER'] = MappingDesc(_Button, 4, 'Left Shoulder', joystick)
  d['BUTTON_RIGHT_SHOULDER'] = MappingDesc(_Button, 5, 'Right Shoulder', joystick)
  d['BUTTON_LEFT_TRIGGER'] = MappingDesc(_Button, 6, 'Left Trigger', joystick)
  d['BUTTON_RIGHT_TRIGGER'] = MappingDesc(_Button, 7, 'Right Trigger', joystick)
  d['BUTTON_SELECT'] = MappingDesc(_Button, 8, 'Select', joystick)
  d['BUTTON_START'] = MappingDesc(_Button, 9, 'Start', joystick)
  d['BUTTON_HOME'] = MappingDesc(_Button, 10, 'Home', joystick)
  d['BUTTON_LEFT_STICK_CLICK'] = MappingDesc(_Button, 11, 'Left Stick', joystick)
  d['BUTTON_RIGHT_STICK_CLICK'] = MappingDesc(_Button, 12, 'Right Stick', joystick)
  d['BUTTON_DPAD_UP'] = MappingDesc(_Button, 13, 'D-Pad Up', joystick)
  d['BUTTON_DPAD_DOWN'] = MappingDesc(_Button, 14, 'D-Pad Down', joystick)
  d['BUTTON_DPAD_LEFT'] = MappingDesc(_Button, 15, 'D-Pad Left', joystick)
  d['BUTTON_DPAD_RIGHT'] = MappingDesc(_Button, 16, 'D-Pad Right', joystick)
  return d


def __map_ps4_cuh_zct2u(joystick):
  """
  Proper controller mapping for Sony Playstation 4 Wireless Controller
  Model CUH-ZXT2U

  """

  d = dict()
  d['AXIS_LEFT_STICK_X'] = MappingDesc(_Axis, 0, 'Left Stick (x)', joystick)
  d['AXIS_LEFT_STICK_Y'] = MappingDesc(_Axis, 1, 'Left Stick (y)', joystick)
  d['BUTTON_OPTIONS'] = MappingDesc(_Button, 9, 'Options', joystick)
  d['BUTTON_LEFT_TRIGGER'] = MappingDesc(_Button, 4, 'Left Trigger', joystick)
  d['BUTTON_RIGHT_TRIGGER'] = MappingDesc(_Button, 5, 'Right Trigger', joystick)
  d['BUTTON_SHARE'] = MappingDesc(_Button, 8, 'Share', joystick)
  if (sys.platform == 'darwin' or sys.platform == 'win32'):
    d['AXIS_LEFT_TRIGGER'] = MappingDesc(_Axis, 3, 'Left Trigger', joystick)
    d['AXIS_RIGHT_TRIGGER'] = MappingDesc(_Axis, 4, 'Right Trigger', joystick)
    d['AXIS_RIGHT_STICK_X'] = MappingDesc(_Axis, 2, 'Right Stick (x)', joystick)
    d['AXIS_RIGHT_STICK_Y'] = MappingDesc(_Axis, 5, 'Right Stick (y)', joystick)
    d['BUTTON_SQUARE'] = MappingDesc(_Button, 0, 'Square', joystick)
    d['BUTTON_CIRCLE'] = MappingDesc(_Button, 2, 'Circle', joystick)
    d['BUTTON_TRIANGLE'] = MappingDesc(_Button, 3, 'Triangle', joystick)
    d['BUTTON_X'] = MappingDesc(_Button, 1, 'X', joystick)
    d['BUTTON_LEFT_STICK_CLICK'] = MappingDesc(_Button, 10, 'Left Stick', joystick)
    d['BUTTON_TOUCHPAD'] = MappingDesc(_Button, 13, 'Touchpad', joystick)
  elif sys.platform.startswith('linux'):
    d['AXIS_LEFT_TRIGGER'] = MappingDesc(_Axis, 2, 'Left Trigger', joystick)
    d['AXIS_RIGHT_TRIGGER'] = MappingDesc(_Axis, 5, 'Right Trigger', joystick)
    d['AXIS_RIGHT_STICK_X'] = MappingDesc(_Axis, 3, 'Right Stick (x)', joystick)
    d['AXIS_RIGHT_STICK_Y'] = MappingDesc(_Axis, 4, 'Right Stick (y)', joystick)
    d['BUTTON_SQUARE'] = MappingDesc(_Button, 3, 'Square', joystick)
    d['BUTTON_CIRCLE'] = MappingDesc(_Button, 1, 'Circle', joystick)
    d['BUTTON_TRIANGLE'] = MappingDesc(_Button, 2, 'Triangle', joystick)
    d['BUTTON_X'] = MappingDesc(_Button, 0, 'X', joystick)
    d['BUTTON_LEFT_STICK_CLICK'] = MappingDesc(_Button, 11, 'Left Stick', joystick)
    d['BUTTON_TOUCHPAD'] = MappingDesc(_Button, 10, 'Touchpad', joystick)
  else:
    raise RuntimeError('Unknown Operating System/Platform {0}'.format(sys.platform))
  return d

__mappings = {
  '030000004c0500006802000011810000' : __map_ps3_cechzc2u,
  '030000004c050000cc09000011810000' : __map_ps4_cuh_zct2u }


def get_joystick_mapping(joystick):
  guid = joystick.guid
  if guid in __mappings:
    return __mappings[guid](joystick)
  return None

