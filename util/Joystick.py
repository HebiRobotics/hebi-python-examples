#!/usr/bin/env python3

from sdl2 import *
from threading import Lock, Condition

SDL_InitSubSystem(SDL_INIT_JOYSTICK)
SDL_JoystickEventState(SDL_ENABLE)
SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, b"1")


def SDL_JoystickGetGUIDString(guid):
  """See https://github.com/marcusva/py-sdl2/issues/75 for explanation"""
  s = ''
  for g in guid.data:
    s += "{:x}".format(g >> 4)
    s += "{:x}".format(g & 0x0F)
  return s


class Joystick(object):

  __joysticks = dict()
  AXIS = 1
  BALL = 2
  HAT = 3
  BUTTON = 4

  def __init__(self, index):
    self.__joystick = SDL_JoystickOpen(index)
    self.__index = index
    self._as_parameter_ = self.__joystick
    self.__initialize()

  def __initialize(self):

    num_axes = SDL_JoystickNumAxes(self)
    num_balls = SDL_JoystickNumBalls(self)
    num_hats = SDL_JoystickNumHats(self)
    num_buttons = SDL_JoystickNumButtons(self)

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

    class JoystickEventsMap(object):

      def __init__(self, num_axes, num_balls, num_hats, num_buttons):
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
        return self.__dict[key][index]

      def __getitem__(self, item1, item2):
        return self.get_event(item1, item2)

    self.__events = JoystickEventsMap(num_axes, num_balls, num_hats, num_buttons)

    from ctypes import byref, c_int

    if num_axes > 0:
      self.__axes_last_val = [None] * num_axes
      for i in range(num_axes):
        self.__axes_last_val[i] = self.__get_axis_state(SDL_JoystickGetAxis(self, i))
    else:
      self.__axes_last_val = []

    if num_balls > 0:
      self.__balls_last_val = [None] * num_balls
      xrel = c_int()
      yrel = c_int()
      for i in range(num_balls):
        SDL_JoystickGetBall(self, i, byref(xrel), byref(yrel))
        self.__balls_last_val[i] = (xrel.value, yrel.value)
    else:
      self.__balls_last_val = []

    if num_hats > 0:
      self.__hats_last_val = [None] * num_hats
      for i in range(num_hats):
        self.__hats_last_val[i] = self.__get_hat_state(SDL_JoystickGetHat(self, i))
    else:
      self.__hats_last_val = []

    if num_buttons > 0:
      self.__buttons_last_val = [None] * num_buttons
      for i in range(num_buttons):
        self.__buttons_last_val[i] = self.__get_button_state(SDL_JoystickGetButton(self, i))
    else:
      self.__buttons_last_val = []

  def __get_axis_state(self, value):
    # Normalize from SDL's signed 16 bit int representation to [-1, 1]
    return value/32767.0

  def __get_hat_state(self, value):
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

  def __get_button_state(self, value):
    return value == SDL_PRESSED

  def _on_axis_motion(self, timestamp, axis, value):
    value = self.__get_axis_state(value)
    event = self.__events[Joystick.AXIS, axis]
    event.acquire()
    self.__axes_last_val[axis] = value
    event.notify_all()
    event.release()
    event(timestamp, value)

  def _on_ball_motion(self, timestamp, ball, xrel, yrel):
    event = self.__events[Joystick.BALL, ball]
    event.acquire()
    self.__balls_last_val[ball] = (xrel, yrel)
    event.notify_all()
    event.release()
    event(timestamp, xrel, yrel)

  def _on_hat_motion(self, timestamp, hat, value):
    hx, hy = self.__get_hat_state(value)
    event = self.__events[Joystick.HAT, hat]
    event.acquire()
    self.__hats_last_val[hat] = (hx, hy)
    event.notify_all()
    event.release()
    event(timestamp, hx, hy)

  def _on_button_event(self, timestamp, button, value):
    pressed = self.__get_button_state(value)
    event = self.__events[Joystick.BUTTON, button]
    event.acquire()
    self.__buttons_last_val[button] = pressed
    event.notify_all()
    event.release()
    event(timestamp, pressed)

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
    return sdl2.ext.compat.stringify(SDL_JoystickName(self), 'utf8')

  @property
  def guid(self):
    _guid = SDL_JoystickGetGUID(self)
    return SDL_JoystickGetGUIDString(_guid)

  def get_axis(self, axis):
    return self.__axes_last_val[axis]

  def get_ball(self, ball):
    return self.__balls_last_val[ball]

  def get_hat(self, hat):
    return self.__hats_last_val[hat]

  def get_button(self, button):
    return self.__buttons_last_val[button]

  def get_next_axis_state(self, axis, timeout=None):
    event = self.__events[Joystick.AXIS, axis]
    event.acquire()
    event.wait(timeout)
    val = self.__axes_last_val[axis]
    event.release()
    return val

  def get_next_ball_state(self, ball, timeout=None):
    event = self.__events[Joystick.BALL, ball]
    event.acquire()
    event.wait(timeout)
    val = self.__balls_last_val[ball]
    event.release()
    return val

  def get_next_hat_state(self, hat, timeout=None):
    event = self.__events[Joystick.HAT, hat]
    event.acquire()
    event.wait(timeout)
    val = self.__hats_last_val[hat]
    event.release()
    return val

  def get_next_button_state(self, button, timeout=None):
    event = self.__events[Joystick.BUTTON, button]
    event.acquire()
    event.wait(timeout)
    val = self.__buttons_last_val[button]
    event.release()
    return val

