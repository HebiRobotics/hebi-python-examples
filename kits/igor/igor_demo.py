from .structure import Igor

igor = Igor()
global keep_running
keep_running = True

def stop_running_callback(*args):
  global keep_running
  keep_running = False


igor.start()

from ..util import Joystick

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

