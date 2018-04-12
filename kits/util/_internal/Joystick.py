#!/usr/bin/env python3

from sdl2 import *
from threading import Lock, Condition
from .sdl_utils import SDL_JoystickGetGUIDString
from .type_utils import assert_callable, assert_type, assert_prange

SDL_InitSubSystem(SDL_INIT_JOYSTICK)
SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER)
SDL_InitSubSystem(SDL_INIT_HAPTIC)
SDL_JoystickEventState(SDL_ENABLE)
SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, b"1")


# ------------------------------------------------------------------------------
# Helper classes
# ------------------------------------------------------------------------------

class JoystickEvent(object):
  """
  TODO: Document
  """

  def __init__(self):
    self.__callbacks = set()
    self.__cv = Condition(Lock())

  def __call__(self, *args):
    for callback in self.__callbacks:
      callback(*args)

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
  TODO: Document
  """

  def __init__(self, num_axes, num_balls, num_hats, num_buttons):

    d = dict()
    d[AXIS] = [None] * num_axes
    for i in range(num_axes):
      d[AXIS][i] = JoystickEvent()

    d[BALL] = [None] * num_balls
    for i in range(num_balls):
      d[BALL][i] = JoystickEvent()

    d[HAT] = [None] * num_hats
    for i in range(num_hats):
      d[HAT][i] = JoystickEvent()

    d[BUTTON] = [None] * num_buttons
    for i in range(num_buttons):
      d[BUTTON][i] = JoystickEvent()

    self.__dict = d

  def get_event(self, key, index):
    """
    TODO: Document

    :param key:
    :type key:  int
    :param index:
    :type index: int

    :rtype: JoystickEvent
    """
    assert_type(index, int, 'index')
    assert_type(key, int, 'key')
    if not key in self.__dict:
      raise KeyError('{0} is not an event type'.format(key))
    events = self.__dict[key]
    assert_prange(index, len(events))
    return events[index]

  def __getitem__(self, item):
    """
    Calls :meth:`get_event` with the given input

    :param item: tuple of key and index
    :type item:  tuple
    """
    assert_type(item, tuple, 'item')
    return self.get_event(*item)


class GameControllerException(RuntimeError):
  def __init__(self, *args, **kwargs):
    super(GameControllerException, self).__init__(*args, **kwargs)


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


def hat_get_val(value):
  hx = 0
  hy = 0
  if value & SDL_HAT_UP:
    hy = 1
  elif value & SDL_HAT_DOWN:
    hy = -1
  if value & SDL_HAT_RIGHT:
    hx = 1
  elif value & SDL_HAT_LEFT:
    hx = -1
  return (hx, hy)


AXIS = 1
BALL = 2
HAT = 3
BUTTON = 4

_joysticks = dict()
_val_mapper = {
  AXIS : lambda value: value*0.0000305185, # value/32767.0
  BALL : lambda xrel,yrel: (xrel,yrel),
  HAT : hat_get_val,
  BUTTON : lambda value: value == SDL_PRESSED }


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

  def __init__(self, index):
    self.__gamepad = SDL_GameControllerOpen(index)

    if not self.__gamepad:
      self.__joystick = None
      self._as_parameter_ = self.__joystick
      raise GameControllerException('Not a game controller')

    self.__joystick = SDL_JoystickOpen(index)
    self.__index = index
    self._as_parameter_ = self.__joystick
    self.__initialize()

    from .JoystickMappings import get_joystick_mapping
    self.__joystick_mapping = get_joystick_mapping(self)

  def __del__(self):
    if (self.__gamepad):
      SDL_GameControllerClose(self.__gamepad)
    if (self.__joystick):
      SDL_JoystickClose(self.__joystick)

  def __getattr__(self, item):
    if item == '__joystick_mapping':
      raise RuntimeError
    elif self.__joystick_mapping == None:
      raise AttributeError
    elif item.startswith('BUTTON_') or item.startswith('AXIS_'):
      return self.__joystick_mapping[item]()
    else:
      raise AttributeError

  def __dir__(self):
    s_dir = set()
    for entry in super(Joystick, self).__dir__():
      s_dir.add(entry)

    if self.__joystick_mapping != None:
      s_dir = s_dir.union(self.__joystick_mapping.axis_ids)
      s_dir = s_dir.union(self.__joystick_mapping.button_ids)
    return [entry for entry in s_dir]

  def __initialize(self):

    num_axes = SDL_JoystickNumAxes(self)
    num_balls = SDL_JoystickNumBalls(self)
    num_hats = SDL_JoystickNumHats(self)
    num_buttons = SDL_JoystickNumButtons(self)

    self.__num_axes = num_axes
    self.__num_balls = num_balls
    self.__num_hats = num_hats
    self.__num_buttons = num_buttons
    self.__events = JoystickEventsMap(num_axes, num_balls, num_hats, num_buttons)

    from ctypes import byref, c_int

    if num_axes > 0:
      axes_last_val = [None] * num_axes
      for i in range(num_axes):
        axes_last_val[i] = _val_mapper[AXIS](SDL_JoystickGetAxis(self, i))
    else:
      axes_last_val = []

    if num_balls > 0:
      balls_last_val = [None] * num_balls
      xrel = c_int()
      yrel = c_int()
      for i in range(num_balls):
        SDL_JoystickGetBall(self, i, byref(xrel), byref(yrel))
        balls_last_val[i] = _val_mapper[BALL](xrel.value, yrel.value)
    else:
      balls_last_val = []

    if num_hats > 0:
      hats_last_val = [None] * num_hats
      for i in range(num_hats):
        hats_last_val[i] = _val_mapper[HAT](SDL_JoystickGetHat(self, i))
    else:
      hats_last_val = []

    if num_buttons > 0:
      buttons_last_val = [None] * num_buttons
      for i in range(num_buttons):
        buttons_last_val[i] = _val_mapper[BUTTON](SDL_JoystickGetButton(self, i))
    else:
      buttons_last_val = []

    last_vals = {
      AXIS : axes_last_val,
      BALL : balls_last_val,
      HAT : hats_last_val,
      BUTTON : buttons_last_val }
    self.__last_vals = last_vals

  def __get_next_val(self, key, idx, timeout):
    event = self.__events[key, idx]
    event.acquire()
    event.wait(timeout)
    val = self.__last_vals[key][idx]
    event.release()
    return val

  def __update_last_val(self, idx, key, *args):
    cvtr = _val_mapper[key]
    value = cvtr(*args)
    event = self.__events[key, idx]
    event.acquire()
    self.__last_vals[key][idx] = value
    event.notify_all()
    event.release()
    return event, value

  def __register_event_handler(self, idx, key, callback):
    self.__events[key, idx].add_event_handler(callback)

  def _on_axis_motion(self, ts, axis, value):
    event, val = self.__update_last_val(axis, AXIS, value)
    event(ts, val)

  def _on_ball_motion(self, ts, ball, xrel, yrel):
    event, val = self.__update_last_val(ball, BALL, xrel, yrel)
    event(ts, *val)

  def _on_hat_motion(self, ts, hat, value):
    event, val = self.__update_last_val(hat, HAT, value)
    event(ts, *val)

  def _on_button_event(self, ts, button, value):
    event, val = self.__update_last_val(button, BUTTON, value)
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
    if not index in _joysticks:
      raise KeyError('Joystick {0} does not exist'.format(index))
    return _joysticks[index]

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
    return sdl2.ext.compat.stringify(SDL_GameControllerName(self.__gamepad), 'utf8')

  @property
  def guid(self):
    """
    :return: the GUID of the device this joystick represents
    :rtype:  str
    """
    return SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(self))

  def add_dpad_event_handler(self, handler):
    """
    FIXME - do dpads get names? do they get indices, or do all controllers only have one?
    TODO: document
    :param handler:
    :return:
    """
    assert_callable(handler)
    self.__register_event_handler(0, HAT, handler)

  def add_axis_event_handler(self, axis, handler):
    """
    TODO

    :param axis: TODO
    :type axis:  int, str
    :param handler: TODO

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError: If `handler` is not callable or
                       `axis` is not an int or str
    :raises KeyError:  If `axis` is a str and is not a valid axis
    :raises IndexError: If `axis` is an int and is out of range
    """
    assert_callable(handler)
    if type(axis) == str:
      axis = self.__joystick_mapping.get_axis(axis).index
    assert_type(axis, int, 'axis')
    self.__register_event_handler(axis, AXIS, handler)

  def add_button_event_handler(self, button, handler):
    """
    TODO

    :param button: TODO
    :type button:  int, str
    :param handler: TODO

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError: If `handler` is not callable or
                       `button` is not an int or str
    :raises KeyError:  If `button` is a str and is not a valid button
    :raises IndexError: If `button` is an int and is out of range
    """
    assert_callable(handler)
    if type(button) == str:
      button = self.__joystick_mapping.get_button(button).index
    assert_type(button, int, 'button')
    self.__register_event_handler(button, BUTTON, handler)

  def get_button_name(self, index):
    """
    Get the name of the button at the given index. If this joystick has no
    mapping, then an exception will be thrown.

    :param index: The index of the button
    :type index:  int

    :return: The human readable name of the button
    :rtype:  str

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError:               If `index` is not an int
    """
    assert_type(index, int, 'index')
    if self.__joystick_mapping == None:
      raise GameControllerException('Joystick has no mapping')
    return self.__joystick_mapping.get_button(index).name

  def get_axis(self, axis):
    """
    Retrieves the last value of the axis provided. This call does not block.

    :param axis: the index or name of the axis. If this argument is a string,
                 it is case insensitive.
    :type axis:  int, str

    :rtype: float

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError: If `axis` is not an int or str
    :raises IndexError: If `axis` is an invalid index
    """
    if (type(axis) == str):
      if self.__joystick_mapping != None:
        return self.__joystick_mapping.get_axis_value(axis)
      else:
        raise GameControllerException('Joystick has no mapping. You must retrieve button values by integer index')
    assert_type(axis, int, 'axis')
    vals = self.__last_vals[AXIS]
    assert_prange(axis, len(vals), 'axis index')
    return vals[axis]

  def get_ball(self, ball):
    """
    Retrieves the last value of the ball at the given index.
    This call does not block.

    :return: TODO
    :rtype:  tuple

    :raises TypeError: If `ball` is not an int
    :raises IndexError: If `ball` is an invalid index
    """
    assert_type(ball, int, 'ball')
    vals = self.__last_vals[BALL]
    assert_prange(ball, len(vals), 'ball index')
    return vals[ball]

  def get_hat(self, hat):
    """
    Retrieves the last value of the hat at the given index.
    This call does not block.

    :param hat:   The index of the hat
    :type button: int

    :return: TODO
    :rtype:  tuple

    :raises TypeError: If `hat` is not an int
    :raises IndexError: If `hat` is an invalid index
    """
    assert_type(hat, int, 'hat')
    vals = self.__last_vals[HAT]
    assert_prange(hat, len(vals), 'hat index')
    return vals[hat]

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
    if (type(button) == str):
      if self.__joystick_mapping != None:
        return self.__joystick_mapping.get_button_value(button)
      else:
        raise GameControllerException('Joystick has no mapping. You must retrieve button values by integer index')
    assert_type(button, int, 'button')
    vals = self.__last_vals[BUTTON]
    assert_prange(button, len(vals), 'button index')
    return vals[button]

  def get_next_axis_state(self, axis, timeout=None):
    """
    Retrieves the next value of the given axis. This call blocks until
    an sdl2.SDL_JOYAXISMOTION event has been received for the axis

    :param axis: TODO
    :type axis:  int, str

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError: If `axis` is not an int or str
    :raises IndexError: If `axis` is an invalid index
    """
    return self.__get_next_val(AXIS, axis, timeout)

  def get_next_ball_state(self, ball, timeout=None):
    """
    Retrieves the next value of the given ball. This call blocks until
    an sdl2.SDL_JOYBALLMOTION event has been received for the ball

    :param ball: TODO
    :type ball:  int

    :raises TypeError: If `ball` is not an int
    :raises IndexError: If `ball` is an invalid index
    """
    return self.__get_next_val(BALL, ball, timeout)

  def get_next_hat_state(self, hat, timeout=None):
    """
    Retrieves the next value of the given hat. This call blocks until
    an sdl2.SDL_JOYHATMOTION event has been received for the hat

    :param hat: TODO
    :type hat:  int

    :raises TypeError: If `hat` is not an int
    :raises IndexError: If `hat` is an invalid index
    """
    return self.__get_next_val(HAT, hat, timeout)

  def get_next_button_state(self, button, timeout=None):
    """
    Retrieves the next value of the given button. This call blocks until
    an sdl2.SDL_JOYBUTTONDOWN or sdl2.SDL_JOYBUTTONUP event has been received
    for the button

    :param button: TODO
    :type button:  int, str

    :raises GameControllerException: If the joystick does not have a mapping
    :raises TypeError: If `button` is not an int or str
    :raises IndexError: If `button` is an invalid index
    """
    return self.__get_next_val(BUTTON, button, timeout)

