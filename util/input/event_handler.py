from sdl2 import *
import sdl2.ext
import threading

from time import sleep, time


class SDLEventHandler(object):

  __sdl_event_attr_dict = {
    SDL_CONTROLLERAXISMOTION : "caxis",
    SDL_CONTROLLERBUTTONDOWN : "cbutton",
    SDL_CONTROLLERBUTTONUP : "cbutton",
    SDL_CONTROLLERDEVICEADDED : "cdevice",
    SDL_CONTROLLERDEVICEREMOVED : "cdevice",
    SDL_CONTROLLERDEVICEREMAPPED : "cdevice"}

  def __init__(self):
    hooks = dict()
    hooks[SDL_CONTROLLERAXISMOTION] = [_joystick_axis_motion]
    hooks[SDL_CONTROLLERBUTTONDOWN] = [_joystick_button_event]
    hooks[SDL_CONTROLLERBUTTONUP] = [_joystick_button_event]
    hooks[SDL_CONTROLLERDEVICEADDED] = [_joystick_added]
    hooks[SDL_CONTROLLERDEVICEREMOVED] = [_joystick_removed]

    self.__event_hooks = hooks
    self.__loop_mutex = threading.Lock()
    self.__last_event_loop_time = 0.0
    self.__event_loop_frequency = 250.0 # Limit event loop to 250 Hz
    self.__event_loop_period = 1.0/self.__event_loop_frequency
    self.__thread = None

  def __dispatch_event(self, sdl_event):
    event = sdl_event.type
    if event not in SDLEventHandler.__sdl_event_attr_dict:
      return

    data = getattr(sdl_event, SDLEventHandler.__sdl_event_attr_dict[event])
    for entry in self.__event_hooks[event]:
      entry(data)

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


from . import joystick as JoystickModule
from .joystick import Joystick as JoystickController
import sdl2.ext.compat


def _joystick_added(sdl_event):
  joystick_id = sdl_event.which
  try:
    joystick = JoystickController(joystick_id)
    JoystickModule._joysticks[joystick_id] = joystick
  except Exception as e:
    import sdl2.ext.compat
    print('WARNING: Caught Exception\n{0}'.format(e))
    print('Could not add Joystick "{0}"'.format(sdl2.ext.compat.stringify(SDL_JoystickNameForIndex(joystick_id), 'utf8')))


def _joystick_removed(sdl_event):
  joystick_id = sdl_event.which
  try:
    import sdl2.ext.compat
    print("Warning: joystick {0} was removed".format(sdl2.ext.compat.stringify(SDL_JoystickNameForIndex(joystick_id), 'utf8')))
  finally:
    pass


def _joystick_axis_motion(sdl_event):
  joystick_id = sdl_event.which
  ts = sdl_event.timestamp
  axis = sdl_event.axis
  value = sdl_event.value
  JoystickController.at_index(joystick_id)._on_axis_motion(ts, axis, value)


def _joystick_button_event(sdl_event):
  joystick_id = sdl_event.which
  ts = sdl_event.timestamp
  button = sdl_event.button
  value = sdl_event.state
  JoystickController.at_index(joystick_id)._on_button_event(ts, button, value)


_singleton = SDLEventHandler()
_singleton.start()
