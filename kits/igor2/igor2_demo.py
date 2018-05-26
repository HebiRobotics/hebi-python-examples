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


def x(*args):
  print('X')

def circle(*args):
  print('CIRCLE')

def triangle(*args):
  print('TRIANGLE')

def square(*args):
  print('SQUARE')

def l1(*args):
  print('L1')

def l2(*args):
  print('L2')

def r1(*args):
  print('R1')

def r2(*args):
  print('R2')


igor.start()

# The joystick has been initialized once `igor.start()` returns
joy = igor.joystick
if joy is None:
  raise RuntimeError('No Joystick found')

joy.add_button_event_handler('SHARE', stop_running_callback)
joy.add_button_event_handler('X', x)
joy.add_button_event_handler('CIRCLE', circle)
joy.add_button_event_handler('TRIANGLE', triangle)
joy.add_button_event_handler('SQUARE', square)
joy.add_button_event_handler('L1', l1)
joy.add_button_event_handler('L2', l2)
joy.add_button_event_handler('R1', r1)
joy.add_button_event_handler('R2', r2)

from time import sleep
while(keep_running):
  sleep(1.0)
