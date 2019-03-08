from sdl2 import *
from threading import Lock, Condition
from ..type_utils import assert_callable, assert_type, assert_prange

SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC)
SDL_JoystickEventState(SDL_ENABLE)
SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, b"1")


# ------------------------------------------------------------------------------
# Misc
# ------------------------------------------------------------------------------


def SDL_JoystickGetGUIDString(guid):
  """See https://github.com/marcusva/py-sdl2/issues/75 for explanation"""
  s = ''
  for g in guid.data:
    s += "{:x}".format(g >> 4)
    s += "{:x}".format(g & 0x0F)
  return s


# ------------------------------------------------------------------------------
# Helper classes
# ------------------------------------------------------------------------------


class JoystickEvent(object):
  """
  Represents an event from a joystick.
  This also holds callbacks to respond to this event.
  Additionally, this acts as a condition variable, providing mutual exclusion to this event.
  """

  __slots__ = ('__callbacks', '__cv')

  def __init__(self):
    self.__callbacks = set()
    self.__cv = Condition(Lock())

  def __call__(self, ts, val):
    for callback in self.__callbacks:
      callback(ts, val)

  def add_event_handler(self, callback):
    """
    Add an event handler to the given event
    """
    # May need to ensure that the input is hashable/has __eq__ & __cmp__
    self.__callbacks.add(callback)

  def notify_all(self):
    self.__cv.notify_all()

  def wait(self, timeout=None):
    self.__cv.wait(timeout)

  def acquire(self):
    self.__cv.acquire()

  def release(self):
    self.__cv.release()


class JoystickEventsMap(object):
  """
  Map used to encapsulate all events for a joystick
  """

  __slots__ = ('_axis_events', '_button_events')

  def __init__(self, num_axes, num_buttons):
    axis_events   = [None] * num_axes
    button_events = [None] * num_buttons

    for i in range(num_axes):
      axis_events[i] = JoystickEvent()

    for i in range(num_buttons):
      button_events[i] = JoystickEvent()

    self._axis_events = axis_events
    self._button_events = button_events

  def get_axis_event(self, index):
    return self._axis_events[index]

  def get_button_event(self, index):
    return self._button_events[index]


class GameControllerException(RuntimeError):
  def __init__(self, *args, **kwargs):
    super(GameControllerException, self).__init__(*args, **kwargs)


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


_joysticks = dict()
_axis_val_cvt = lambda value: value*0.0000305185
_button_val_cvt = lambda value: value != 0


# ------------------------------------------------------------------------------
# Joystick Class
# ------------------------------------------------------------------------------


class Joystick(object):
  """
  Represents a joystick input device. This class is used to receive
  input events from the user. Both asynchronous and synchronous facilities
  are exposed.

  This class wraps the SDL2 library.
  """

  # TODO
  #__slots__ = []

  def __init__(self, index):
    self.__gamepad = SDL_GameControllerOpen(index)

    if not self.__gamepad:
      raise GameControllerException('Not a game controller')

    self.__joystick = SDL_JoystickOpen(index)
    self.__index = index
    self._as_parameter_ = self.__gamepad

    # See details at https://wiki.libsdl.org/SDL_GameControllerAxis
    num_axes = 6
    # See details at https://wiki.libsdl.org/SDL_GameControllerButton
    num_buttons = 15
    self.__initialize_values(num_axes, num_buttons)

    from ._joystick_mapping import default_joystick_mapping
    self.__joystick_mapping = default_joystick_mapping()

  def __del__(self):
    if self.__gamepad:
      SDL_GameControllerClose(self.__gamepad)
    if self.__joystick:
      SDL_JoystickClose(self.__joystick)

  def __initialize_values(self, num_axes, num_buttons):
    self.__events = JoystickEventsMap(num_axes, num_buttons)
    from ctypes import byref, c_int

    last_axis_vals = [None] * num_axes
    for i in range(num_axes):
      last_axis_vals[i] = _axis_val_cvt(SDL_GameControllerGetAxis(self, i))
    
    last_button_vals = [None] * num_buttons
    for i in range(num_buttons):
      last_button_vals[i] = _button_val_cvt(SDL_GameControllerGetButton(self, i))

    self.__last_axis_vals = last_axis_vals
    self.__last_button_vals = last_button_vals

  def __get_next_axis_val(self, idx, timeout):
    event = self.__events.get_axis_event(idx)
    event.acquire()
    event.wait(timeout)
    value = self.__last_axis_vals[idx]
    event.release()
    return value

  def __get_next_button_val(self, idx, timeout):
    event = self.__events.get_button_event(idx)
    event.acquire()
    event.wait(timeout)
    value = self.__last_button_vals[idx]
    event.release()
    return value

  def __update_last_axis_val(self, idx, val):
    value = _axis_val_cvt(val)
    event = self.__events.get_axis_event(idx)
    event.acquire()
    self.__last_axis_vals[idx] = value
    event.notify_all()
    event.release()
    return event, value

  def __update_last_button_val(self, idx, val):
    value = _button_val_cvt(val)
    event = self.__events.get_button_event(idx)
    event.acquire()
    self.__last_button_vals[idx] = value
    event.notify_all()
    event.release()
    return event, value

  def _on_axis_motion(self, ts, axis, value):
    event, val = self.__update_last_axis_val(axis, value)
    event(ts, val)

  def _on_button_event(self, ts, button, value):
    event, val = self.__update_last_button_val(button, value)
    event(ts, val)

  @staticmethod
  def joystick_count():
    """
    :return: The number of SDL Joysticks found
    :rtype:  int
    """
    return SDL_NumJoysticks()

  @staticmethod
  def at_index(index):
    """
    :type index: int
    :return: the Joystick at the given index
    :rtype: Joystick

    :raises TypeError: If `index` is not an int
    :raises KeyError:  If `index` is not a joystick
    """
    assert_type(index, int, 'index')
    if index not in _joysticks:
      raise KeyError('Joystick {0} does not exist'.format(index))
    return _joysticks[index]

  @staticmethod
  def available_joysticks():
    """
    Retrieve all available joysticks. Each element in the returned list
    is a pair of the index and name of the joystick.

    :return: a list of (int, str) pairs
    :rtype:  list
    """
    ret = list()
    for i in range(Joystick.joystick_count()):
      try:
        joy = Joystick.at_index(i)
        ret.append((i, joy.name))
      except:
        pass
    return ret

  @property
  def index(self):
    """
    :return: The index of this Joystick
    :rtype:  int
    """
    return self.__index

  @property
  def name(self):
    """
    :return: The name of the joystick
    :rtype:  str
    """
    import sdl2.ext.compat
    return sdl2.ext.compat.stringify(SDL_GameControllerName(self), 'utf8')

  @property
  def guid(self):
    """
    :return: the GUID of the device this joystick represents
    :rtype:  str
    """
    return SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(self.__joystick))

  @property
  def controller_type(self):
    return 'GameController'

  def add_axis_event_handler(self, axis, handler):
    """
    :param axis:    
    :type axis:     int, str
    :param handler: 

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError:  If `handler` is not callable
                        or `axis` is not an int or str
    :raises KeyError:   If `axis` is a str and is not a valid axis
    :raises IndexError: If `axis` is an int and is out of range
    """
    assert_callable(handler)
    if type(axis) == str:
      axis = self.__joystick_mapping.get_axis(axis)
    assert_type(axis, int, 'axis')
    self.__events.get_axis_event(axis).add_event_handler(handler)

  def add_button_event_handler(self, button, handler):
    """
    :param button:  
    :type button:   int, str
    :param handler: 

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError:  If `handler` is not callable
                        or `button` is not an int or str
    :raises KeyError:   If `button` is a str and is not a valid button
    :raises IndexError: If `button` is an int and is out of range
    """
    assert_callable(handler)
    if type(button) == str:
      button = self.__joystick_mapping.get_button(button)
    assert_type(button, int, 'button')
    self.__events.get_button_event(button).add_event_handler(handler)

  def get_axis(self, axis):
    """
    Retrieves the last value of the axis provided. This call does not block.

    :param axis: the index or name of the axis. If this argument is a string,
                 it is case insensitive.
    :type axis:  int, str
    
    :rtype: float

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError:  If `axis` is not an int or str
    :raises IndexError: If `axis` is an invalid index
    """
    if type(axis) == str:
      axis = self.__joystick_mapping.get_axis(axis)
    return self.__last_axis_vals[axis]

  def get_button(self, button):
    """
    Retrieves the last value of the button. This call does not block.

    :param button: The index or name of the button. If this argument is
                   a string, it is case insensitive.
    :type button:  int, str
    
    :return: The most recent value received by the event loop of the button.
             ``True`` corresponds to the button being pressed.
    :rtype:  bool

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError:               If `button` is not an int or str
    :raises IndexError:              If `button` is an invalid (int) index
    """
    if type(button) == str:
      button = self.__joystick_mapping.get_button(button)
    return self.__last_button_vals[button]

  def get_next_axis_state(self, axis, timeout=None):
    """
    Retrieves the next value of the given axis. This call blocks until
    an sdl2.<joy motion> event has been received for the axis

    :param axis: 
    :type axis:  int, str

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError: If `axis` is not an int or str
    :raises IndexError: If `axis` is an invalid index
    """
    if type(axis) == str:
      axis = self.__joystick_mapping.get_axis(axis)
    return self.__get_next_axis_val(axis, timeout)

  def get_next_button_state(self, button, timeout=None):
    """
    Retrieves the next value of the given button. This call blocks until
    an sdl2.<button down> or sdl2.<button up> event has been received
    for the button

    :param button: 
    :type button:  int, str

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError: If `button` is not an int or str
    :raises IndexError: If `button` is an invalid index
    """
    if type(button) == str:
      button = self.__joystick_mapping.get_button(button)
    return self.__get_next_button_val(button, timeout)
