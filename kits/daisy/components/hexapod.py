import os
import numpy as np
import threading

from hebi.trajectory import create_trajectory
from math import atan2
from numpy import float32, float64, nan, sign
from numpy.linalg import det, norm, svd
from time import sleep, time

import hebi
from .joystick_interface import _add_event_handlers
from .leg import Leg

step_first_legs = (0, 3, 4)

is_main_thread_active = lambda: threading.main_thread().is_alive()

import typing

if typing.TYPE_CHECKING:
    from typing import Callable
    import numpy.typing as npt
    from hebi._internal.trajectory import Trajectory
    from hebi._internal.graphics import Color
    from .configuration import HexapodConfig, Parameters
    T = typing.TypeVar('T')


def retry_on_error(func: 'Callable[[], T]', on_error_func=None, sleep_time=0.1):
    """Call the input function until it succeeds, sleeping on failure by the
    specified amount."""
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


def create_group(config: 'HexapodConfig'):
    """Used by :class:`Hexapod` to create the group to interface with modules.

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

            group.feedback_frequency = config.robot_feedback_frequency
            group.command_lifetime = config.robot_command_lifetime
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
        '_mobile_io', '_rotation_velocity', '_translation_velocity',
        # High level kinematic state of the entire robot
        '_foot_forces', '_gravity_dir', '_vel_xyz',
        # Timestamps
        '_last_feedback_time', '_last_time', '_start_time', '_stop_time',
        # Fields which compose the robot
        # Leg state
        '_legs', '_active_flight_legs',
        # Misc
        '_mode', '_chassis_mass'
    )

    def __init__(self, config: 'HexapodConfig', params: 'Parameters'):
        self._config = config
        self._params = params
        self._started = False
        self._quit_flag = False
        self._num_spins = 0
        self._mode = 'step'  # TODO: Should there be a 'startup' mode as well?
        # Identifies which set of 3 legs to enable walking
        self._active_flight_legs = 1

        self._foot_forces = np.zeros((3, 6), dtype=float64)
        self._gravity_dir = np.array([0.0, 0.0, -1.0], dtype=float64)
        self._translation_velocity = np.zeros(3, dtype=float64)
        self._rotation_velocity = np.zeros(3, dtype=float64)
        self._last_feedback_time = 0.0  # TODO: Is this still needed?

        self._on_stop_callbacks = list()
        self._on_startup_callbacks = list()

        from threading import Lock
        # Used to synchronize input commands
        self._input_lock = Lock()
        self._state_lock = Lock()

    def _wait_for_feedback(self):
        self._group.get_next_feedback(reuse_fbk=self._group_feedback)

        # compute gravity vector
        avg_gravity = np.zeros(3, dtype=float32)

        for leg in self._legs:
            local_gravity = leg.get_local_gravity()
            if np.isfinite(local_gravity).all():
                avg_gravity += local_gravity

        self._gravity_dir[:] = avg_gravity / norm(avg_gravity)

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

    def _on_startup(self):
        for entry in self._on_startup_callbacks:
            entry()

    def _stop(self):
        """Stop running.

        This happens once the user requests the demo to stop, or when
        the running application begins to shut down.
        """
        print('Shutting down...')
        duration = self._stop_time - self._start_time
        tics = float(self._num_spins)
        avg_frequency = tics / duration
        print(f'Ran for: {duration} seconds.')
        print(f'Average processing frequency: {avg_frequency} Hz')

        # Set the LED color strategy back to the default
        self._group_command.clear()
        self._group_command.led.color = 'transparent'
        self._group.send_command_with_acknowledgement(self._group_command)

        self._on_stop()

    def _create_startup_trajectories(self, startup_seconds):
        # Note: It is very unclear what the C++ code is doing in the `first_run` block here.
        # It appears that the intention is that `startup_trajectory` uses the initial state of the trajectory from the leg's `step` variable
        #start_time = time()
        #get_relative_time = lambda: time() - start_time

        num_joints = Leg.joint_count
        times = [0.0, startup_seconds * 0.25, startup_seconds * 0.5, startup_seconds * 0.75, startup_seconds]

        trajectories: 'list[Trajectory]' = []

        positions = np.empty((num_joints, 5))
        velocities = np.zeros((num_joints, 5))
        accelerations = np.zeros((num_joints, 5))

        for i, leg in enumerate(self._legs):
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

            trajectories.append(create_trajectory(times, positions, velocities, accelerations))
        return trajectories

    def _soft_startup(self, first_run, startup_seconds):

        self._group.send_feedback_request()
        self._wait_for_feedback()

        startup_trajectories = self._create_startup_trajectories(startup_seconds)

        foot_forces = self._foot_forces

        # User callbacks invoked here
        if first_run: self._on_startup()

        start_time = time()
        get_relative_time = lambda: time() - start_time
        t = get_relative_time()

        while t < startup_seconds:
            self._wait_for_feedback()
            t = get_relative_time()
            # Track the trajectories generated
            for i in range(6):
                self._legs[i].track_trajectory(t, startup_trajectories[i], self._gravity_dir * 9.8, foot_forces[:, i])

            self.send_command()

    def _spin_once(self):
        self._wait_for_feedback()

        foot_forces = self._foot_forces
        gravity = self._gravity_dir * 9.8

        current_time = time()
        self.update_stance(current_time - self._last_time)

        if self.need_to_step():
            self.start_step(current_time)

        self.update_steps(current_time)
        self.compute_foot_forces(current_time, foot_forces)

        for i, leg in enumerate(self._legs):
            leg.compute_state(current_time, gravity, foot_forces[:, i])

        self.send_command()
        self._last_time = current_time

    def _start(self):
        """Main processing method.

        This runs on a background thread.
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
                    _add_event_handlers(self, self.mobile_io, self.config.controller_mapping)
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
        """Send a request to stop the demo. If the demo has not been started,
        this method raises an exception.

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

            group = create_group(self._config)

            num_modules = group.size
            self._group = group
            self._auxilary_group_command = hebi.GroupCommand(num_modules)
            self._group_command = hebi.GroupCommand(num_modules)
            self._group_feedback = hebi.GroupFeedback(num_modules)
            self._group_info = hebi.GroupInfo(num_modules)

            # Construction of legs

            # The legs need to start with valid feedback, so we must wait for a feedback here
            self._group.get_next_feedback(reuse_fbk=self._group_feedback)

            kin = hebi.robot_model.import_from_hrdf(os.path.join(self._params.resource_directory, 'daisy.hrdf'))

            legs: 'list[Leg]' = list()
            for i in range(6):
                leg_kinematics = kin.get_subtree_with_root(tag=f'leg{i+1}')
                if leg_kinematics is None:
                    raise RuntimeError(f'Could not find Leg #{i} in daisy.hrdf')
                start_idx = leg_kinematics.dof_count * i
                indices = [start_idx, start_idx + 1, start_idx + 2]
                leg = Leg(i, leg_kinematics, self._params, self._group_command.create_view(indices), self._group_feedback.create_view(indices))
                legs.append(leg)

            # HACK: The mass is hardcoded as 21Kg.
            mass = 21.0

            self._legs = legs
            self._chassis_mass = mass

            setup_controller = self._config.setup_controller

            while True:
                try:
                    mio = setup_controller()
                    if mio is not None:
                        self._mobile_io = mio
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

    def toggle_mode(self):
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

        feet_xyz_com = feet_xyz.mean(axis=1)  # rowwise
        base_xyz_com = base_xyz.mean(axis=1)  # rowwise

        for i in range(6):
            feet_xyz[:, i] -= feet_xyz_com
            base_xyz[:, i] -= base_xyz_com

        # SVD of weighted correlation matrix
        xyz_correlation = base_xyz @ feet_xyz.T
        u, scaling, vh = svd(xyz_correlation)

        s = np.identity(3)
        product_det = det(u @ vh)
        s[2, 2] = sign(product_det)

        ret = np.identity(4)
        ret[:3, :3] = vh.T @ s @ u.T
        ret[:3, 3] = feet_xyz_com - base_xyz_com

        return ret

    def need_to_step(self):
        if self._mode == 'stance' or self.is_stepping():
            return False

        shift_pose = self.get_body_pose_from_feet()
        stance_shift = shift_pose[:3, -1:]  # top right corner

        # Rotated too much
        yaw = abs(atan2(shift_pose[1, 0], shift_pose[0, 0]))
        if yaw > self._params.step_threshold_rotate:
            return True

        # Shifted too much in translation
        position_norm = norm(stance_shift[:2])
        velocity_norm = norm(self._vel_xyz[:2])
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
            leg.update_step(t)

    def get_leg(self, index: int):
        return self._legs[index]

    def compute_foot_forces(self, t, foot_forces):
        foot_stance_radius = np.zeros(6, dtype=float64)
        legs = self._legs

        def normalize(x: 'npt.NDArray[np.float64]'): return x / x.sum()

        contact_factors = np.empty(6, dtype=float64)
        stance_radii = np.empty(6, dtype=float64)
        for i in range(6):
            contact_factors[i] = legs[i].get_foot_contact_factor(t)
            stance_radii[i] = legs[i].get_stance_radius(t, self._gravity_dir)

        # Redistribute weight to just modules in stance
        chassis_accel = 3.3 * np.sin(np.pi * np.mean(contact_factors[contact_factors < 1]))
        robot_mass = self._chassis_mass + legs_in_air_mass
        chassis_force =  robot_mass * (-9.8 * self._gravity_dir + chassis_accel)
        foot_forces[0:3, :] = normalize(contact_factors / stance_radii) * chassis_force 
    def clear_leg_colors(self):
        cmd = self._auxilary_group_command
        cmd.clear()
        cmd.led.color = 'transparent'
        self._group.send_command(cmd)

    def set_leg_color(self, index, color: 'Color'):
        cmd = self._auxilary_group_command
        cmd.clear()

        for idx in self.get_leg(index)._command_view._indices:
            cmd[idx].led.led = color

        self._group.send_command(cmd)

    def set_translation_velocity(self, x, y, z):
        with self._input_lock:
            self._translation_velocity[0] = x
            self._translation_velocity[1] = y
            self._translation_velocity[2] = z

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
    def mobile_io(self):
        return self._mobile_io

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
