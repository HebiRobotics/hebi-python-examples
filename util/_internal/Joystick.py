#!/usr/bin/env python3

from sdl2 import *
from threading import Lock, Condition

SDL_InitSubSystem(SDL_INIT_JOYSTICK)
SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER)
SDL_InitSubSystem(SDL_INIT_HAPTIC)
SDL_JoystickEventState(SDL_ENABLE)
SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, b"1")


def SDL_JoystickGetGUIDString(guid):
  """See https://github.com/marcusva/py-sdl2/issues/75 for explanation"""
  s = ''
  for g in guid.data:
    s += "{:x}".format(g >> 4)
    s += "{:x}".format(g & 0x0F)
  return s


def _hat_get_val(value):
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


class _JoystickEventsMap(object):

  def __init__(self, num_axes, num_balls, num_hats, num_buttons):

    class JoystickEvent(object):

      def __init__(self):
        self.__callbacks = []
        self.__cv = Condition(Lock())

      def __call__(self, *args):
        for callback in self.__callbacks:
          callback(*args)

      def notify_all(self):
        self.__cv.notify_all()

      def wait(self, timeout=None):
        self.__cv.wait(timeout)

      def acquire(self):
        self.__cv.acquire()

      def release(self):
        self.__cv.release()

    d = dict()
    d[Joystick.AXIS] = [None] * num_axes
    for i in range(num_axes):
      d[Joystick.AXIS][i] = JoystickEvent()

    d[Joystick.BALL] = [None] * num_balls
    for i in range(num_balls):
      d[Joystick.BALL][i] = JoystickEvent()

    d[Joystick.HAT] = [None] * num_hats
    for i in range(num_hats):
      d[Joystick.HAT][i] = JoystickEvent()

    d[Joystick.BUTTON] = [None] * num_buttons
    for i in range(num_buttons):
      d[Joystick.BUTTON][i] = JoystickEvent()

    self.__dict = d

  def get_event(self, key, index):
    if not key in self.__dict:
      raise KeyError('{0} is not in the dictionary'.format(key))
    elif index >= len(self.__dict[key]):
      raise IndexError('index {0} >= {1}'.format(index, len(self.__dict[key])))
    elif index < 0:
      raise IndexError('index {0} must be greater than zero'.format(index))
    return self.__dict[key][index]

  def __getitem__(self, item):
    return self.get_event(*item)


class GameControllerException(RuntimeError):
  def __init__(self, *args, **kwargs):
    super(GameControllerException, self).__init__(*args, **kwargs)


class Joystick(object):

  __joysticks = dict()
  AXIS = 1
  BALL = 2
  HAT = 3
  BUTTON = 4

  # Converts input arguments (from SDL event) to value(s) this api uses externally
  # For instance,
  #   Joystick/Gamepad button state is either pressed or not (true/false),
  #   and this is mapped from `value == SDL_PRESSED`
  __val_mapper = {
    AXIS : lambda value: value*0.0000305185, # value/32767.0
    BALL : lambda xrel,yrel: (xrel,yrel),
    HAT : _hat_get_val,
    BUTTON : lambda value: value == SDL_PRESSED }

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

  def __del__(self):
    if (self.__gamepad):
      SDL_GameControllerClose(self.__gamepad)
    if (self.__joystick):
      SDL_JoystickClose(self.__joystick)

  def __initialize(self):

    num_axes = SDL_JoystickNumAxes(self)
    num_balls = SDL_JoystickNumBalls(self)
    num_hats = SDL_JoystickNumHats(self)
    num_buttons = SDL_JoystickNumButtons(self)

    self.__events = _JoystickEventsMap(num_axes, num_balls, num_hats, num_buttons)

    from ctypes import byref, c_int

    if num_axes > 0:
      axes_last_val = [None] * num_axes
      for i in range(num_axes):
        axes_last_val[i] = Joystick.__val_mapper[Joystick.AXIS](SDL_JoystickGetAxis(self, i))
    else:
      axes_last_val = []

    if num_balls > 0:
      balls_last_val = [None] * num_balls
      xrel = c_int()
      yrel = c_int()
      for i in range(num_balls):
        SDL_JoystickGetBall(self, i, byref(xrel), byref(yrel))
        balls_last_val[i] = Joystick.__val_mapper[Joystick.BALL](xrel.value, yrel.value)
    else:
      balls_last_val = []

    if num_hats > 0:
      hats_last_val = [None] * num_hats
      for i in range(num_hats):
        hats_last_val[i] = Joystick.__val_mapper[Joystick.HAT](SDL_JoystickGetHat(self, i))
    else:
      hats_last_val = []

    if num_buttons > 0:
      buttons_last_val = [None] * num_buttons
      for i in range(num_buttons):
        buttons_last_val[i] = Joystick.__val_mapper[Joystick.BUTTON](SDL_JoystickGetButton(self, i))
    else:
      buttons_last_val = []

    last_vals = {
      Joystick.AXIS : axes_last_val,
      Joystick.BALL : balls_last_val,
      Joystick.HAT : hats_last_val,
      Joystick.BUTTON : buttons_last_val }
    self.__last_vals = last_vals

  def __get_next_val(self, key, idx, timeout):
    event = self.__events[key, idx]
    event.acquire()
    event.wait(timeout)
    val = self.__last_vals[key][idx]
    event.release()
    return val

  def __update_last_val(self, idx, key, *args):
    cvtr = Joystick.__val_mapper[key]
    value = cvtr(*args)
    event = self.__events[key, idx]
    event.acquire()
    self.__last_vals[key][idx] = value
    event.notify_all()
    event.release()
    return event, value

  def _on_axis_motion(self, ts, axis, value):
    event, val = self.__update_last_val(axis, Joystick.AXIS, value)
    event(ts, val)

  def _on_ball_motion(self, ts, ball, xrel, yrel):
    event, val = self.__update_last_val(ball, Joystick.BALL, xrel, yrel)
    event(ts, *val)

  def _on_hat_motion(self, ts, hat, value):
    event, val = self.__update_last_val(hat, Joystick.HAT, value)
    event(ts, *val)

  def _on_button_event(self, ts, button, value):
    event, val = self.__update_last_val(button, Joystick.BUTTON, value)
    event(ts, val)

  @staticmethod
  def _set_at(index, joystick):
    Joystick.__joysticks[index] = joystick

  @staticmethod
  def at_index(index):
    if not index in Joystick.__joysticks:
      raise KeyError('Joystick {0} does not exist'.format(index))
    return Joystick.__joysticks[index]

  @property
  def index(self):
    return self.__index

  @property
  def name(self):
    import sdl2.ext.compat
    return sdl2.ext.compat.stringify(SDL_GameControllerName(self.__gamepad), 'utf8')

  @property
  def guid(self):
    _guid = SDL_JoystickGetGUID(self)
    return SDL_JoystickGetGUIDString(_guid)

  def get_axis(self, axis):
    vals = self.__last_vals[Joystick.AXIS]
    if (axis < 0 or axis >= len(vals)):
      raise IndexError('axis index out of range. [0,{0}) are valid inputs'.format(len(vals)))
    return vals[axis]

  def get_ball(self, ball):
    vals = self.__last_vals[Joystick.BALL]
    if (ball < 0 or ball >= len(vals)):
      raise IndexError('ball index out of range. [0,{0}) are valid inputs'.format(len(vals)))
    return vals[ball]

  def get_hat(self, hat):
    vals = self.__last_vals[Joystick.HAT]
    if (hat < 0 or hat >= len(vals)):
      raise IndexError('hat index out of range. [0,{0}) are valid inputs'.format(len(vals)))
    return vals[hat]

  def get_button(self, button):
    vals = self.__last_vals[Joystick.BUTTON]
    if (button < 0 or button >= len(vals)):
      raise IndexError('button index out of range. [0,{0}) are valid inputs'.format(len(vals)))
    return vals[button]

  def get_next_axis_state(self, axis, timeout=None):
    return self.__get_next_val(Joystick.AXIS, axis, timeout)

  def get_next_ball_state(self, ball, timeout=None):
    return self.__get_next_val(Joystick.BALL, ball, timeout)

  def get_next_hat_state(self, hat, timeout=None):
    return self.__get_next_val(Joystick.HAT, hat, timeout)

  def get_next_button_state(self, button, timeout=None):
    return self.__get_next_val(Joystick.BUTTON, button, timeout)

