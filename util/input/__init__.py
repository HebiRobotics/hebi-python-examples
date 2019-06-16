
import os

# PySDL2 on Windows requires you to set the path to the folder where SDL2.dll resides.
# If the user has not set the SDL directory in the `PYSDL2_DLL_PATH` environment variable,
# then this function is invoked.
def __inject_sdl2_win():
  from platform import machine
  cpu = machine()
  lib_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'lib'))

  if cpu == 'AMD64':
    lib_path = os.path.join(lib_base_path, 'win_x64')
  elif cpu == 'x86':
    lib_path = os.path.join(lib_base_path, 'win_x86')
  else:
    raise RuntimeError('Unsupported Platform {0}'.format(cpu))
  os.environ['PYSDL2_DLL_PATH'] = lib_path


if os.name == 'nt':
  if 'PYSDL2_DLL_PATH' not in os.environ:
    __inject_sdl2_win()


from . import event_handler as __event_handler


def register_event(event, callback):
  """
  Register a callback for the given event
  """
  try:
    __event_handler._singleton.register_event(event, callback)
  except Exception as e:
    print('Caught exception when attempting to register event')
    print(str(e))


def register_key_event_handler(key, callback):
  """
  :param callback: must accept 3 parameters -
                    `ts` (timestamp),
                    `state` (pressed or not) and
                    `repeat` (n times key has been repeatedly pressed)
  """
  from .keyboard import _kbd_instance
  _kbd_instance.add_key_event_handler(key, callback)


def listen_for_escape_key():
  """
  Used to automatically listen for any presses of the `ESC` key in a background thread.
  Note: the thread on which `ESC` events will be listened is spawned regardless of this function.
  """
  keyboard._listen_for_esc()


def has_esc_been_pressed():
  """
  Used to see if `ESC` key has been pressed. Nonblocking.
  """
  return keyboard._has_esc_been_pressed()


def clear_esc_state():
  """
  Clear the flag corresponding to `ESC` being pressed. Nonblocking.

  Returns the old value of the flag.
  """
  return keyboard._clear_esc_state()


from .joystick import Joystick
from . import keyboard
