#!/usr/bin/env python3

import os
import sys


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


import numpy as np
import hebi
from time import time
from components import arm_container
from components import trajectory_time_heuristic
from hebi.util import create_mobile_io


class Waypoint(object):

  def __init__(self, num_modules):
    self._position = np.empty(num_modules, np.float64)
    self._velocity = np.empty(num_modules, np.float64)
    self._acceleration = np.empty(num_modules, np.float64)

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

  @property
  def number_of_waypoints(self):
    return len(self._waypoints)

  def lock(self):
    self._mutex.acquire()

  def unlock(self):
    self._mutex.release()


def build_trajectory(state):
  # Using the current state object, replan through the waypoints using the trajectory generator
  num_modules = state.arm.group.size

  # Reuse the first waypoint as the last one by adding it to the end.
  state._waypoints.append(state._waypoints[0])

  # Build trajectory
  num_waypoints = len(state._waypoints)
  positions = np.empty((num_modules, num_waypoints), dtype=np.float64)
  velocities = np.empty((num_modules, num_waypoints), dtype=np.float64)
  accelerations = np.empty((num_modules, num_waypoints), dtype=np.float64)

  for i in range(num_waypoints):
    waypoint = state._waypoints[i]
    positions[:, i] = waypoint.position
    velocities[:, i] = waypoint.velocity
    accelerations[:, i] = waypoint.acceleration

  time_vector = trajectory_time_heuristic.get_times(positions, velocities, accelerations)
  return hebi.trajectory.create_trajectory(time_vector, positions, velocities, accelerations)


def command_proc(state):
  # The background thread in which communication with the modules occur
  group = state.arm.group
  group.feedback_frequency = 100.0

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
    command.effort = state.arm.get_efforts(feedback)

    current_mode = state.mode

    if current_mode == 'playback':
      if prev_mode != 'playback':
        # First time
        trajectory = build_trajectory(state)
        start_time = time()

      time_in_seconds = time() - start_time
      if time_in_seconds > trajectory.duration:
        start_time = time()
        time_in_seconds = 0

      pos, vel, acc = trajectory.get_state(time_in_seconds)
      command.position = pos
      command.velocity = vel
    elif current_mode == 'training' and prev_mode != 'training':
      # Clear old position commands
      command.position = None
      command.velocity = None
    group.send_command(command)
    state.unlock()
    prev_mode = current_mode


def add_waypoint(state, stop):
  if state.number_of_waypoints == 0:
    stop = True

  num_modules = state.current_position.size

  if stop:
    vel_accel_val = 0.0
  else:
    vel_accel_val = np.nan

  waypoint = Waypoint(num_modules)
  pos = waypoint.position
  vel = waypoint.velocity
  acc = waypoint.acceleration
  pos[:] = state.current_position
  vel[:] = vel_accel_val
  acc[:] = vel_accel_val
  state._waypoints.append(waypoint)


def clear_waypoints(state):
  state._waypoints = list()


def print_and_cr(msg):
  sys.stdout.write(msg + '\r\n')


def run():
  arm = arm_container.create_3_dof()
  state = State(arm)

  from threading import Thread
  cmd_thread = Thread(target=command_proc,
                      name='Command Thread',
                      args=(state,))
  cmd_thread.start()

  print_and_cr("Press 'b2' to add waypoint ('b3' for stopping at this waypoint), 'b4' to clear waypoints, 'b5' to playback, and 'b1' to quit.")
  print_and_cr("When in playback mode, 'b6' resumes training, and 'b1' quits.")

  # Mobile device setup
  phone_family = 'HEBI'
  phone_name   = "mobileIO"

  lookup = hebi.Lookup()
  sleep(2)

  print('Waiting for Mobile IO device to come online...')
  m = create_mobile_io(lookup, phone_family, phone_name)
  m.update()

  while not m.get_button_state(1):
    # Update MobileIO state
    if not m.update():
      print("Failed to get feedback from MobileIO")
      continue

    state.lock()

    current_mode = state.mode

    if current_mode == 'training':
      m.set_led_color("blue")
      if m.get_button_state(2):
        add_waypoint(state, False)
      elif m.get_button_state(3):
        add_waypoint(state, True)
      elif m.get_button_state(4):
        clear_waypoints(state)
      elif m.get_button_state(5):
        if state.number_of_waypoints > 1:
          state._mode = 'playback'
        else:
          print_and_cr('Need at least two waypoints to enter playback mode!')
    elif current_mode == 'playback':
      m.set_led_color("green")
      if m.get_button_state(6):
        state._mode = 'training'

    state.unlock()

  m.set_led_color("red")
  state._quit = True
  print_and_cr('')


run()
