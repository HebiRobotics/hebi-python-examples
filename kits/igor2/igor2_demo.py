import os
import sys

# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path


from components.igor import Igor
from util.input import Joystick
igor = Igor()
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
