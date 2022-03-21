import numpy as np
import threading

from hebi.trajectory import create_trajectory
from math import atan2, pi, sin
from numpy import float32, float64, nan, sign
from numpy.linalg import det, norm, svd
from time import sleep, time
from util.math_utils import quat2rot

import hebi
from .joystick_interface import register_hexapod_event_handlers
from .leg import Leg

step_first_legs = (0, 3, 4)

is_main_thread_active = lambda: threading.main_thread().is_alive()


def retry_on_error(func, on_error_func=None, sleep_time=0.1):
  """
  Call the input function until it succeeds,
  sleeping on failure by the specified amount.
  """
  if not callable(func):
    raise TypeError()
  while True:
    try:
      ret = func()
      return ret
    except:
      if on_error_func is not None:
        on_error_func()
      sleep(sleep_time)


def create_group(config):
  """
  Used by :class:`Hexapod` to create the group to interface with modules

  :param config:     The runtime configuration
  """
  imitation = config.is_imitation

  if imitation:
    num_modules = len(config.module_names)
    from hebi.util import create_imitation_group
    return create_imitation_group(num_modules)
  else:
    names = config.module_names
    families = [config.family]
    lookup = hebi.Lookup()

    def connect():
      group = lookup.get_group_from_names(families, names)
      if group is None:
        print('Cannot find hexapod on network...')
        raise RuntimeError()
      elif group.size != len(names):
        raise RuntimeError()
      return group

    # Let the lookup object discover modules, before trying to connect
    sleep(2.0)
    return retry_on_error(connect)


def load_gains(hexapod):
  group = hexapod.group

  # Bail out if group is imitation
  if hexapod.config.is_imitation:
    return

  gains_command = hebi.GroupCommand(group.size)
  sleep(0.1)

  import os
  gains_xml = os.path.join(hexapod.config.resources_directory, 'gains18.xml')

  try:
    gains_command.read_gains(gains_xml)
  except Exception as e:
    print(f'Warning - Could not load gains: {e}')
    return

  # Send gains multiple times
  for i in range(3):
    group.send_command(gains_command)
    sleep(0.1)


_active_legs = [
  [0, 3, 4],
  [1, 2, 5]
]


class Hexapod:

  # TODO: Sort categorically
  __slots__ = (
    # Configuration variables
    '_config', '_params',
    # HEBI group interaction
    '_group', '_group_command', '_group_feedback', '_group_info',
    '_auxilary_group_command',
    # Lifecycle state
    '_input_lock', '_state_lock', '_proc_thread', '_quit_flag', '_started',
    '_on_stop_callbacks', '_on_startup_callbacks', '_num_spins',
    # Fields whjch pertain to controlling demo
    '_joy', '_rotation_velocity', '_translation_velocity',
    # High level kinematic state of the entire robot
    '_foot_forces', '_gravity', '_vel_xyz',
    # Timestamps
    '_last_feedback_time', '_last_time', '_start_time', '_stop_time',
    # Fields which compose the robot
    # Leg state
    '_legs', '_active_flight_legs',
    # Misc
    '_mode', '_startup_trajectories', '_weight'
  )

  def __init__(self, config, params):
    self._config = config
    self._params = params
    self._started = False
    self._quit_flag = False
    self._num_spins = 0
    self._mode = 'step' # TODO: Should there be a 'startup' mode as well?
    # Identifies which set of 3 legs to enable walking
    self._active_flight_legs = 1

    self._foot_forces = np.zeros((3, 6), dtype=float64)
    self._gravity = np.array([0.0, 0.0, -1.0], dtype=float64)
    self._translation_velocity = np.zeros(3, dtype=float64)
    self._rotation_velocity = np.zeros(3, dtype=float64)
    self._last_feedback_time = 0.0 # TODO: Is this still needed?

    self._on_stop_callbacks = list()
    self._on_startup_callbacks = list()

    from threading import Lock
    # Used to synchronize input commands
    self._input_lock = Lock()
    self._state_lock = Lock()

  def _wait_for_feedback(self):
    legs = self._legs
    self._group.get_next_feedback(reuse_fbk=self._group_feedback)

    # compute gravity vector
    down = np.array([0.0, 0.0, -1.0], dtype=float32)
    avg_gravity = np.zeros(3, dtype=float32)

    rot_matrix = np.identity(3, dtype=float64)

    for i in range(6):
      leg = legs[i]
      orientation = leg._feedback_view.orientation[0]

      local_gravity = leg.kinematics.base_frame[0:3, 0:3] @ quat2rot(orientation, rot_matrix).T @ down

      if np.isfinite(local_gravity).all():
        avg_gravity += local_gravity

    avg_gravity /= norm(avg_gravity)
    self._gravity[:] = avg_gravity

  def _should_continue(self):
    """
    Contract: This method assumes the caller has acquired `_state_lock`
    """
    if not is_main_thread_active() or self._quit_flag:
      return False
    return True

  def _on_stop(self):
    for entry in self._on_stop_callbacks:
      entry()

  def _on_startup(self, first_run):
    for entry in self._on_startup_callbacks:
      entry(first_run)

  def _stop(self):
    """
    Stop running. This happens once the user requests the demo to stop,
    or when the running application begins to shut down.
    """
    print('Shutting down...')
    duration = self._stop_time - self._start_time
    tics = float(self._num_spins)
    avg_frequency = tics/duration
    print('Ran for: {0} seconds.'.format(duration))
    print('Average processing frequency: {0} Hz'.format(avg_frequency))
    
    # Set the LED color strategy back to the default
    self._group_command.clear()
    self._group_command.led.color = 'transparent'
    self._group.send_command_with_acknowledgement(self._group_command)

    self._on_stop()

  def _create_startup_trajectories(self, startup_seconds):
    legs = self._legs

    # Note: It is very unclear what the C++ code is doing in the `first_run` block here.
    # It appears that the intention is that `startup_trajectory` uses the initial state of the trajectory from the leg's `step` variable
    #start_time = time()
    #get_relative_time = lambda: time() - start_time

    num_joints = Leg.joint_count
    times = [0.0, startup_seconds*0.25, startup_seconds*0.5, startup_seconds*0.75, startup_seconds]

    positions = np.empty((num_joints, 5))
    velocities = np.zeros((num_joints, 5))
    accelerations = np.zeros((num_joints, 5))

    startup_trajectories: 'list[hebi.trajectory._Trajectory]' = []
    for i in range(6):
      leg = legs[i]
      leg_start = leg._feedback_view.position
      leg_end, v = leg.get_state_at_time(0)

      # HACK: hardcoded as offset from leg end
      leg_mid = leg_end.copy()
      leg_mid[1] -= 0.3
      leg_mid[2] -= 0.15

      step_first = i in step_first_legs

      positions[:, 0] = leg_start
      positions[:, 1] = leg_mid if step_first else leg_start
      positions[:, 2] = leg_end if step_first else leg_start
      positions[:, 3] = leg_end if step_first else leg_mid
      positions[:, 4] = leg_end

      velocities[:, 1] = nan
      velocities[:, 3] = nan
      accelerations[:, 1] = nan
      accelerations[:, 3] = nan

      startup_trajectories.append(create_trajectory(times, positions, velocities, accelerations))
    return startup_trajectories

  def _soft_startup(self, first_run, startup_seconds):
    group = self._group
    group_feedback = self._group_feedback
    group_command = self._group_command

    group.send_feedback_request()
    self._wait_for_feedback()

    startup_trajectories = self._create_startup_trajectories(startup_seconds)

    foot_forces = self._foot_forces

    # User callbacks invoked here
    self._on_startup(first_run)

    legs = self._legs

    start_time = time()
    get_relative_time = lambda: time() - start_time

    while True:
      self._wait_for_feedback()
      gravity = self._gravity * 9.8
      t = get_relative_time()
      # Track the trajectories generated
      for i in range(6):
        legs[i].track_trajectory(t, startup_trajectories[i], gravity, foot_forces[:, i])

      self.send_command()
      if not t < startup_seconds:
        break

  def _spin_once(self):
    group = self._group
    group_feedback = self._group_feedback
    group_command = self._group_command

    start_time = self._start_time
    self._wait_for_feedback()

    foot_forces = self._foot_forces
    gravity = self._gravity * 9.8

    current_time = time()
    self.update_stance(current_time - self._last_time)

    if self.need_to_step():
      self.start_step(current_time)

    self.update_steps(current_time)
    self.compute_foot_forces(current_time, foot_forces)

    legs = self._legs
    for i in range(6):
      leg = legs[i]
      leg.compute_state(current_time)
      leg.compute_torques(gravity, foot_forces[:, i])
      # Make sure to send position, velocity and effort
      leg.set_command()

    self.send_command()
    self._last_time = current_time

  def _start(self):
    """
    Main processing method. This runs on a background thread.
    """
    first_run = True

    # TODO: This seems strange. See if it should be increased/decreased.
    startup_seconds = 4.5

    while True:
      self._soft_startup(first_run, startup_seconds)

      # Delay registering event handlers until now, so the Hexapod
      # can start up without being interrupted by user commands.
      # View this function in `event_handlers.py` to see
      # all of the joystick event handlers registered
      if first_run:
        try:
          register_hexapod_event_handlers(self)
        except Exception as e:
          print('Caught exception while registering event handlers:\n{0}'.format(e))
        self._start_time = time()
        self._last_time = self._start_time
        first_run = False

      self._state_lock.acquire()
      while self._should_continue():
        self._state_lock.release()
        self._spin_once()
        self._num_spins += 1
        self._state_lock.acquire()

      if self._quit_flag:
        self._stop_time = time()
        self._state_lock.release()
        self._stop()
        # Break out if quit was requested
        return
      else:
        self._state_lock.release()

  def add_on_stop_callback(self, callback):
    self._on_stop_callbacks.append(callback)

  def add_on_startup_callback(self, callback):
    self._on_startup_callbacks.append(callback)

  def request_stop(self):
    """
    Send a request to stop the demo.
    If the demo has not been started, this method raises an exception.

    :raises RuntimeError: if `start()` has not been called
    """
    self._state_lock.acquire()
    # TODO: self._ensure_started()
    self._quit_flag = True
    self._state_lock.release()

  def start(self):
    with self._state_lock:
      if self._started:
        return

      config = self._config
      group = create_group(config)
      group.feedback_frequency = config.robot_feedback_frequency
      group.command_lifetime = config.robot_command_lifetime

      num_modules = group.size
      self._group = group
      self._auxilary_group_command = hebi.GroupCommand(num_modules)
      self._group_command = hebi.GroupCommand(num_modules)
      self._group_feedback = hebi.GroupFeedback(num_modules)
      self._group_info = hebi.GroupInfo(num_modules)

      # Construction of legs
      from math import radians
      leg_angles = [radians(entry) for entry in [30.0, -30.0, 90.0, -90.0, 150.0, -150.0]]
      leg_distances = [0.2375, 0.2375, 0.1875, 0.1875, 0.2375, 0.2375]
      leg_configs = ['left', 'right'] * 3
      num_joints_in_leg = Leg.joint_count
      parameters = self._params

      # HACK: The mass is hardcoded as 21Kg. This hardcodes the weight to be the force here (which will be accurate as long as we are on Earth).
      weight = 21.0 * 9.8

      # The legs need to start with valid feedback, so we must wait for a feedback here
      self._group.get_next_feedback(reuse_fbk=self._group_feedback)

      legs = list()
      for i, angle, distance, leg_configuration in zip(range(num_modules), leg_angles, leg_distances, leg_configs):
        start_idx = num_joints_in_leg * i
        indices = [start_idx, start_idx + 1, start_idx + 2]
        leg = Leg(i, angle, distance, parameters, leg_configuration, self._group_command.create_view(indices), self._group_feedback.create_view(indices))
        legs.append(leg)

      self._legs = legs
      self._weight = weight

      joystick_selector = self._config.joystick_selector

      while True:
        try:
          joy = joystick_selector()
          if joy is None:
            raise RuntimeError
          self._joy = joy
          break
        except:
          pass

      load_gains(self)

      from threading import Condition, Lock, Thread
      start_condition = Condition(Lock())

      def start_routine(start_condition):
        with start_condition:
          # Calling thread holds `_state_lock` AND
          # is also blocked until we call `start_condition.notify_all()`
          # So we can safely modify this here.
          self._started = True
        
          # Allow the calling thread to continue
          start_condition.notify_all()

        self._start()

      self._proc_thread = Thread(target=start_routine,
                                 name='Hexapod Controller',
                                 args=(start_condition,))
      
      # We will have started the thread before returning,
      # but make sure the function has begun running before
      # we release the `_state_lock` and return
      with start_condition:
        self._proc_thread.start()
        start_condition.wait()

  def update_stance(self, dt):
    with self._input_lock:
      translation_vel = self._translation_velocity.copy()
      rotation_vel = self._rotation_velocity.copy()

    current_z = self._legs[0].level_home_stance_xyz[2]
    dz = translation_vel[2] * dt
    max_z = self._params.max_z
    min_z = self._params.min_z

    if current_z + dz > max_z:
      translation_vel[2] = (max_z - current_z) / dt
    elif current_z + dz < min_z:
      translation_vel[2] = (min_z - current_z) / dt

    if self._mode == 'step':
      rotation_vel[1] = 0.0

    for leg in self._legs:
      leg.update_stance(translation_vel, rotation_vel, dt)      

    self._vel_xyz = translation_vel.copy()

  def send_command(self):
    return self._group.send_command(self._group_command)

  def update_gains(self):
    group = self._group
    size = group.size

    gains = hebi.GroupCommand(size)
    # TODO: gains_file = "gains{0}.xml".format(size)
    # TODO: gains.read_gains(gains_file)
    return group.send_command_with_acknowledgement(gains)

  def update_mode(self, toggles):
    if self._mode == 'stance':
      self._mode = 'step'
    else:
      self._mode = 'stance'

  def get_body_pose_from_feet(self):
    feet_xyz = np.empty((3, 6), dtype=float64)
    base_xyz = np.empty((3, 6), dtype=float64)

    legs = self._legs

    for i in range(6):
      leg = legs[i]
      feet_xyz[:, i] = leg.fbk_stance_xyz
      base_xyz[:, i] = leg.home_stance_xyz

    feet_xyz_com = feet_xyz.mean(axis=1) # rowwise
    base_xyz_com = base_xyz.mean(axis=1) # rowwise

    for i in range(6):
      feet_xyz[0:3, i] -= feet_xyz_com
      base_xyz[0:3, i] -= base_xyz_com

    # SVD of weighted correlation matrix
    xyz_correlation = base_xyz @ feet_xyz.T
    u, scaling, vh = svd(xyz_correlation)

    s = np.identity(3)
    product_det = det(u @ vh)
    s[2, 2] = sign(product_det)

    ret = np.identity(4)
    ret[0:3, 0:3] = vh.T @ s @ u.T
    ret[0:3, 3] = feet_xyz_com - base_xyz_com

    return ret

  def need_to_step(self):
    if self._mode == 'stance' or self.is_stepping():
      return False

    shift_pose = self.get_body_pose_from_feet()
    stance_shift = shift_pose[0:3, -1:] # top right corner

    # Rotated too much
    yaw = abs(atan2(shift_pose[1, 0], shift_pose[0, 0]))
    if yaw > self._params.step_threshold_rotate:
      return True

    # Shifted too much in translation
    position_norm = norm(stance_shift[0:2])
    velocity_norm = norm(self._vel_xyz[0:2])
    shift = position_norm + velocity_norm * 0.7
    # HACK: `period` is an instance method of `Step`. It's hardcoded there to be 0.7
    if shift > self._params.step_threshold_shift:
      return True

    return False

  def is_stepping(self):
    return any([leg.mode == 'flight' for leg in self._legs])

  def start_step(self, t):
    legs = self._legs
    stepping_legs = _active_legs[self._active_flight_legs]
    for i in stepping_legs:
      leg = legs[i]
      leg.start_step(t)

    # TODO: Clean this up?
    if self._active_flight_legs == 0:
      self._active_flight_legs = 1
    else:
      self._active_flight_legs = 0

  def update_steps(self, t):
    for leg in self._legs:
      if leg.mode == 'flight':
        leg.update_step(t)

  def get_leg(self, index):
    return self._legs[index]

  def compute_foot_forces(self, t, foot_forces):
    factors = np.zeros(6, dtype=float64)
    blend_factors = np.zeros(6, dtype=float64)
    gravity = -self._gravity

    legs = self._legs

    for i in range(6):
      stance = legs[i].cmd_stance_xyz
      dot_product = gravity.dot(stance)
      factors[i] = norm(gravity * dot_product - stance)

    factors_sum = factors.sum()

    for i in range(6):
      factors[i] = factors_sum / factors[i]
      leg = legs[i]

      # Redistribute weight to just modules in stance
      if leg.mode == 'flight':
        switch_time = 0.1
        current_step_time = leg.get_step_time(t)
        step_period = leg.step_period
        if current_step_time < switch_time:
          blend_factors[i] = (switch_time - current_step_time) / switch_time
        elif (step_period - current_step_time) < switch_time:
          blend_factors[i] = (current_step_time - (step_period - switch_time)) / switch_time
      
        factors[i] *= blend_factors[i]
      else:
        blend_factors[i] = 1.0

    factors /= factors.sum()
    weight = self._weight

    for i in range(6):
      factor = factors[i] * (1.0 + 0.33 * sin(pi * blend_factors[i]))
      foot_forces[0:3, i] = factors[i] * weight * gravity

  def clear_leg_colors(self):
    cmd = self._auxilary_group_command
    cmd.clear()
    cmd.led.color = 'transparent'
    self._group.send_command(cmd)

  def set_leg_color(self, index, color):
    group_size = self._group_command.size
    colors = [None] * group_size
    colors[index] = color

    cmd = self._auxilary_group_command
    cmd.clear()
    cmd.led.color = colors
    self._group.send_command(cmd)

  def set_translation_velocity_x(self, value):
    with self._input_lock:
      self._translation_velocity[0] = value

  def set_translation_velocity_y(self, value):
    with self._input_lock:
      self._translation_velocity[1] = value

  def set_translation_velocity_z(self, value):
    with self._input_lock:
      self._translation_velocity[2] = value

  def set_rotation_velocity_y(self, value):
    with self._input_lock:
      self._rotation_velocity[1] = value

  def set_rotation_velocity_z(self, value):
    with self._input_lock:
      self._rotation_velocity[2] = value

  @property
  def group(self):
    return self._group

  @property
  def config(self):
    return self._config

  @property
  def joystick_dead_zone(self):
    return self._params.joystick_dead_zone

  @property
  def joystick(self):
    return self._joy

  @property
  def last_feedback_time(self):
    return self._last_feedback_time

  @property
  def mode(self):
    return self._mode

  @property
  def started(self):
    with self._state_lock:
      return self._started
 
