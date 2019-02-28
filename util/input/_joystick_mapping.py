from sdl2 import *

_default_axis_map = dict()
_default_button_map = dict()

# ------------------------------------------------------------------------------
# Axes
# ------------------------------------------------------------------------------

# LEFTX
_default_axis_map['LEFT_STICK_X'] = SDL_CONTROLLER_AXIS_LEFTX

# LEFTY
_default_axis_map['LEFT_STICK_Y'] = SDL_CONTROLLER_AXIS_LEFTY

# RIGHTX
_default_axis_map['RIGHT_STICK_X'] = SDL_CONTROLLER_AXIS_RIGHTX

# RIGHTY
_default_axis_map['RIGHT_STICK_Y'] = SDL_CONTROLLER_AXIS_RIGHTY

# TRIGGERLEFT
_default_axis_map['LEFT_TRIGGER'] = SDL_CONTROLLER_AXIS_TRIGGERLEFT
_default_axis_map['TRIGGER_LEFT'] = SDL_CONTROLLER_AXIS_TRIGGERLEFT
_default_axis_map['L2'] = SDL_CONTROLLER_AXIS_TRIGGERLEFT

# TRIGGERRIGHT
_default_axis_map['RIGHT_TRIGGER'] = SDL_CONTROLLER_AXIS_TRIGGERRIGHT
_default_axis_map['TRIGGER_RIGHT'] = SDL_CONTROLLER_AXIS_TRIGGERRIGHT
_default_axis_map['R2'] = SDL_CONTROLLER_AXIS_TRIGGERRIGHT

# ------------------------------------------------------------------------------
# Buttons
# ------------------------------------------------------------------------------

# A
_default_button_map['A'] = SDL_CONTROLLER_BUTTON_A
_default_button_map['CROSS'] = SDL_CONTROLLER_BUTTON_A

# B
_default_button_map['B'] = SDL_CONTROLLER_BUTTON_B
_default_button_map['CIRCLE'] = SDL_CONTROLLER_BUTTON_B

# X
_default_button_map['X'] = SDL_CONTROLLER_BUTTON_X
_default_button_map['SQUARE'] = SDL_CONTROLLER_BUTTON_X

# Y
_default_button_map['Y'] = SDL_CONTROLLER_BUTTON_Y
_default_button_map['TRIANGLE'] = SDL_CONTROLLER_BUTTON_Y

# BACK
_default_button_map['BACK'] = SDL_CONTROLLER_BUTTON_BACK
_default_button_map['SELECT'] = SDL_CONTROLLER_BUTTON_BACK
_default_button_map['SHARE'] = SDL_CONTROLLER_BUTTON_BACK

# GUIDE
# Note - the guide button does not exist on all controllers.
# For example, Some Xbox controllers do not have one. Consequently, this shouldn't be used for required events.
_default_button_map['GUIDE'] = SDL_CONTROLLER_BUTTON_GUIDE
_default_button_map['TOUCHPAD'] = SDL_CONTROLLER_BUTTON_GUIDE

# START
_default_button_map['START'] = SDL_CONTROLLER_BUTTON_START
_default_button_map['OPTIONS'] = SDL_CONTROLLER_BUTTON_START

# LEFTSTICK
_default_button_map['LEFT_STICK'] = SDL_CONTROLLER_BUTTON_LEFTSTICK
_default_button_map['L3'] = SDL_CONTROLLER_BUTTON_LEFTSTICK

# RIGHTSTICK
_default_button_map['RIGHT_STICK'] = SDL_CONTROLLER_BUTTON_RIGHTSTICK
_default_button_map['R3'] = SDL_CONTROLLER_BUTTON_RIGHTSTICK

# LEFTSHOULDER
_default_button_map['LEFT_SHOULDER'] = SDL_CONTROLLER_BUTTON_LEFTSHOULDER
_default_button_map['L1'] = SDL_CONTROLLER_BUTTON_LEFTSHOULDER

# RIGHTSHOULDER
_default_button_map['RIGHT_SHOULDER'] = SDL_CONTROLLER_BUTTON_RIGHTSHOULDER
_default_button_map['R1'] = SDL_CONTROLLER_BUTTON_RIGHTSHOULDER

# DPAD_UP
_default_button_map['DPAD_UP'] = SDL_CONTROLLER_BUTTON_DPAD_UP
_default_button_map['UP'] = SDL_CONTROLLER_BUTTON_DPAD_UP

# DPAD_DOWN
_default_button_map['DPAD_DOWN'] = SDL_CONTROLLER_BUTTON_DPAD_DOWN
_default_button_map['DOWN'] = SDL_CONTROLLER_BUTTON_DPAD_DOWN

# DPAD_LEFT
_default_button_map['DPAD_LEFT'] = SDL_CONTROLLER_BUTTON_DPAD_LEFT
_default_button_map['LEFT'] = SDL_CONTROLLER_BUTTON_DPAD_LEFT

# DPAD_RIGHT 
_default_button_map['DPAD_RIGHT'] = SDL_CONTROLLER_BUTTON_DPAD_RIGHT
_default_button_map['RIGHT'] = SDL_CONTROLLER_BUTTON_DPAD_RIGHT


class JoystickMapping(object):
  """Maps string axis and button names to their SDL2 values"""
  __slots__ = ('__axis_map', '__button_map')

  def __init__(self, a_map, b_map):
    self.__axis_map = a_map
    self.__button_map = b_map

  def get_axis(self, axis):
    if axis not in self.__axis_map:
      raise ValueError('Invalid axis {0}'.format(axis))
    return self.__axis_map[axis]

  def get_button(self, button):
    if button not in self.__button_map:
      raise ValueError('Invalid button {0}'.format(button))
    return self.__button_map[button]


_default_mapping = JoystickMapping(_default_axis_map, _default_button_map)


def default_joystick_mapping():
  return _default_mapping
