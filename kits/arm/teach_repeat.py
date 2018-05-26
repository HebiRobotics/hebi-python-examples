import numpy as np
import hebi

from time import time


class Waypoint(object):

  def __init__(self, num_waypoints):
    self._position = np.empty(num_waypoints, np.float64)
    self._velocity = np.empty(num_waypoints, np.float64)
    self._acceleration = np.empty(num_waypoints, np.float64)

  @property
  def position(self):
    return self._position
  
  @property
  def velocity(self):
    return self._velocity

  @property
  def acceleration(self):
    return self._acceleration
  

class State(object):

  def __init__(self, arm):
    self._waypoints = list()
    self._quit = False
    self._mode = 'training'
    self._arm = arm
    self._current_position = np.empty(arm.dof_count, dtype=np.float64)
    from threading import Lock
    self._mutex = Lock()

  @property
  def quit(self):
    return self._quit
  
  @property
  def mode(self):
    return self._mode
  
  @property
  def arm(self):
    return self._arm

  @property
  def current_position(self):
    return self._current_position
  
  def lock(self):
    self._mutex.acquire()

  def unlock(self):
    self._mutex.release()


def command_proc(state):
  group = state.arm.group
  num_modules = group.size

  command = hebi.GroupCommand(num_modules)
  feedback = hebi.GroupFeedback(num_modules)
  prev_mode = state.mode
  start_time = time()
  trajectory = None

  while True:
    if group.get_next_feedback(reuse_fbk=feedback) is None:
      print('Did not receive feedback')
      continue

    state.lock()
    if state.quit:
      state.unlock()
      break

    feedback.get_position(state.current_position)
