import os
import sys

# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path


from components.igor import Igor

igor = Igor()
keep_running = True

def stop_running_callback(*args):
  global keep_running
  keep_running = False


igor.start()

# The joystick has been initialized once `igor.start()` returns
joy = igor.joystick
if joy is None:
  raise RuntimeError('No Joystick found')

from time import sleep
while keep_running:
  sleep(1.0)
