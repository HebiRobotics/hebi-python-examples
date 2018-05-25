from sdl2 import *
import sdl2.ext
import threading

from time import sleep, time


class SDLEventHandler(object):

  __sdl_event_attr_dict = {
    SDL_JOYAXISMOTION : "jaxis",
    SDL_JOYBALLMOTION : "jball",
    SDL_JOYHATMOTION : "jhat",
    SDL_JOYBUTTONDOWN : "jbutton",
    SDL_JOYBUTTONUP : "jbutton",
    SDL_JOYDEVICEADDED : "jdevice",
    SDL_JOYDEVICEREMOVED : "jdevice"}

  def __init__(self):
    hooks = dict()
    hooks[SDL_JOYAXISMOTION] = [_joystick_axis_motion]
    hooks[SDL_JOYBALLMOTION] = [_joystick_ball_motion]
    hooks[SDL_JOYHATMOTION] = [_joystick_hat_motion]
    hooks[SDL_JOYBUTTONDOWN] = [_joystick_button_event]
    hooks[SDL_JOYBUTTONUP] = [_joystick_button_event]
    hooks[SDL_JOYDEVICEADDED] = [_joystick_added]
    hooks[SDL_JOYDEVICEREMOVED] = []

    self.__event_hooks = hooks
    self.__loop_mutex = threading.Lock()
    self.__last_event_loop_time = 0.0

    self.__event_loop_frequency = 250.0 # Limit event loop to 250 Hz
    self.__event_loop_period = 1.0/self.__event_loop_frequency

  def __on_event(self, event, data):
    data = getattr(data, SDLEventHandler.__sdl_event_attr_dict[event])
    for entry in self.__event_hooks[event]:
      entry(data)

  def __dispatch_event(self, sdl_event):
    self.__on_event(sdl_event.type, sdl_event)

  def __run(self):
    while True:
      last_time = self.__last_event_loop_time
      now_time = time()
      dt = now_time - last_time
      loop_period = self.__event_loop_period
      if dt < loop_period:
        sleep(loop_period-dt)

      self.__last_event_loop_time = now_time
      for entry in sdl2.ext.get_events():
        self.__loop_mutex.acquire()
        try:
          self.__dispatch_event(entry)
        except:
          pass
        finally:
          self.__loop_mutex.release()

  def register_event(self, event, callback):
    """
    :param event:
    :param callback:

    :raises TypeError: If ``callback`` is not callable
    """
    if not hasattr(callback, '__call__'):
      raise TypeError('{0} is not callable'.format(callback))
    if event in self.__event_hooks:
      self.__loop_mutex.acquire()
      self.__event_hooks[event].append(callback)
      self.__loop_mutex.release()

  def start(self):
    """
    Start the event handler on a background thread
    """
    self.__thread = threading.Thread(target=self.__run, name="SDL Event Processor")
    self.__thread.daemon = True
    self.__thread.start()


# ------------------------------------------------------------------------------
# Events
# ------------------------------------------------------------------------------


from . import Joystick as JoystickModule
from .Joystick import Joystick, GameControllerException
import sdl2.ext.compat


def _joystick_added(sdl_event):
  joystick_id = sdl_event.which
  try:
    joystick = Joystick(joystick_id)
    JoystickModule._joysticks[joystick_id] = joystick
  except Exception as e:
    import sdl2.ext.compat
    print('WARNING: Caught Exception\n{0}'.format(e))
    print('Could not add Joystick "{0}"'.format(sdl2.ext.compat.stringify(SDL_JoystickNameForIndex(joystick_id), 'utf8')))


def _joystick_axis_motion(sdl_event):
  joystick_id = sdl_event.which
  ts = sdl_event.timestamp
  axis = sdl_event.axis
  value = sdl_event.value
  Joystick.at_index(joystick_id)._on_axis_motion(ts, axis, value)


def _joystick_ball_motion(sdl_event):
  joystick_id = sdl_event.which
  ts = sdl_event.timestamp
  ball = sdl_event.ball
  xrel = sdl_event.xrel
  yrel = sdl_event.yrel
  Joystick.at_index(joystick_id)._on_ball_motion(ts, ball, xrel, yrel)


def _joystick_hat_motion(sdl_event):
  joystick_id = sdl_event.which
  ts = sdl_event.timestamp
  hat = sdl_event.hat
  value = sdl_event.value
  Joystick.at_index(joystick_id)._on_hat_motion(ts, hat, value)


def _joystick_button_event(sdl_event):
  joystick_id = sdl_event.which
  ts = sdl_event.timestamp
  button = sdl_event.button
  value = sdl_event.state
  Joystick.at_index(joystick_id)._on_button_event(ts, button, value)


_singleton = SDLEventHandler()
_singleton.start()
