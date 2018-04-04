__all__ = ['DemoUtils', 'math_func']

from ._internal import EventQueue as __EventQueue
from ._internal import Joystick as __Joystick


def register_event(event, callback):
  try:
    __EventQueue._singleton.register_event(event, callback)
  except Exception as e:
    print('Caught exception when attempting to register event')
    print(str(e))


Joystick =__Joystick.Joystick