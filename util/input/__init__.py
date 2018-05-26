
import os

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


from .joystick import Joystick
