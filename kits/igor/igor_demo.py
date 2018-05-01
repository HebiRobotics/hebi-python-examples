if __name__ == "__main__":
  # Load parent modules
  import os, sys
  here = os.path.dirname(os.path.abspath(__file__))
  parent = os.path.join(here, '..', '..')
  sys.path = [parent] + sys.path

  from importlib import import_module
  import_module('kits')

  from kits.util import Joystick
  from kits.igor.structure import Igor

else:
  from .structure import Igor
  from ..util import Joystick


igor = Igor()
global keep_running
keep_running = True

def stop_running_callback(*args):
  global keep_running
  keep_running = False


igor.start()



joy = None
for i in range(Joystick.joystick_count()):
  try:
    joy = Joystick.at_index(i)
  except:
    pass

if joy is None:
  raise RuntimeError('No Joystick found')

joy.add_button_event_handler('SHARE', stop_running_callback)

from time import sleep
while(keep_running):
  sleep(1.0)

