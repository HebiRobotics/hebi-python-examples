import numpy as np
from numpy import matlib

from .arm import Arm
from .chassis import Chassis
from .leg import Leg
from .joystick_interface import register_igor_event_handlers, label_to_pin_map
from .configuration import Igor2Config

from math import atan2, degrees, radians
from time import sleep, time
from util import math_utils

import hebi
import sys
import os
import threading

import typing
if typing.TYPE_CHECKING:
    from hebi import GroupFeedback

# ------------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------------


# Used for Igor background controller thread
if sys.version_info[0] == 3:
    is_main_thread_active = lambda: threading.main_thread().is_alive()
else:
    is_main_thread_active = lambda: any((i.name == "MainThread") and i.is_alive() for i in threading.enumerate())


def retry_on_error(func, on_error_func=None, sleep_time=0.1):
    """Call the input function until it succeeds, sleeping on failure by the
    specified amount."""
    if not hasattr(func, '__call__'):
        raise TypeError()
    while True:
        try:
            ret = func()
            return ret
        except:
            if on_error_func is not None:
                on_error_func()
            sleep(sleep_time)


def load_gains(igor: 'Igor'):
    group = igor._group

    # Bail out if group is imitation
    if igor._config.is_imitation:
        return

    gains_command = hebi.GroupCommand(group.size)
    sleep(0.1)

    try:
        if igor._has_camera:
            gains_command.read_gains(igor._config.gains_xml)
        else:
            gains_command.read_gains(igor._config.gains_no_camera_xml)
    except Exception as e:
        print(f'Warning - Could not load gains: {e}')
        return

    # Send gains multiple times
    for i in range(3):
        group.send_command(gains_command)
        sleep(0.1)


def create_group(config, has_camera):
    """Used by :class:`Igor` to create the group to interface with modules.

    :param config:     The runtime configuration
    :type config:      .configuration.Igor2Config
    :param has_camera:
    :type has_camera:  bool
    """
    imitation = config.is_imitation

    if imitation:
        if has_camera:
            num_modules = len(config.module_names)
        else:
            num_modules = len(config.module_names_no_cam)

        from hebi.util import create_imitation_group
        return create_imitation_group(num_modules)
    else:
        if has_camera:
            names = config.module_names
        else:
            names = config.module_names_no_cam
        families = [config.family]
        lookup = hebi.Lookup()

        def connect():
            group = lookup.get_group_from_names(families, names)
            if group is None:
                print('Cannot find igor on network...')
                raise RuntimeError()
            elif group.size != len(names):
                raise RuntimeError()
            return group

        # Let the lookup object discover modules, before trying to connect
        sleep(2.0)
        return retry_on_error(connect)


# ------------------------------------------------------------------------------
# Igor Class
# ------------------------------------------------------------------------------


class Igor(object):

    Lean_P = 1.0
    Lean_I = 0.0
    Lean_D = 10.0

# ------------------------------------------------
# Helper functions
# ------------------------------------------------

    def _ensure_started(self):
        """
        Contract: This method assumes the caller has acquired `_state_lock`
        """
        if not self._started:
            self._state_lock.release()
            raise RuntimeError('Igor has not been started (igor.start() was not called)')

    def _should_continue(self):
        """
        Contract: This method assumes the caller has acquired `_state_lock`
        """
        if not is_main_thread_active() or self._quit_flag:
            return False

        if self._transition_to_idle_possible:
            # Check if stance is low enough to transition to idle
            return self._left_leg.knee_angle < 2.5

        return True

# ------------------------------------------------
# Calculations
# ------------------------------------------------

    def _update_com(self):
        """Updates the CoM of all bodies."""

        self._left_arm.update_position()
        self._right_arm.update_position()
        self._left_leg.update_position()
        self._right_leg.update_position()

        coms = self._coms
        coms[0:3, 0] = self.left_leg.com
        coms[0:3, 1] = self.right_leg.com
        coms[0:3, 2] = self.left_arm.com
        coms[0:3, 3] = self.right_arm.com

        masses = self._masses

        self._com = np.sum(np.multiply(coms, matlib.repmat(masses, 3, 1)), axis=1) / self._mass

    def _update_pose_estimate(self, gyro, orientation):
        """The CoM of all bodies has been updated at this point.

        :param gryo:
        :param orientation: [numModules x 4] matrix of orientation
        """

        # Update gyro values from current modules feedback
        # Transposing allows us to do calculations a bit easier below
        self._pose_gyros[:, :] = gyro.T

        # Update imu frames
        # Leg endeffectors
        np.copyto(self._imu_frames[0], self._left_leg.current_tip_fk)
        np.copyto(self._imu_frames[1], self._right_leg.current_tip_fk)
        # Hip link output frames
        np.copyto(self._imu_frames[3], self._left_leg._current_fk[1])
        np.copyto(self._imu_frames[5], self._right_leg._current_fk[1])
        # Arm base bracket
        np.copyto(self._imu_frames[7], self._left_arm._current_fk[1])
        np.copyto(self._imu_frames[11], self._right_arm._current_fk[1])
        # Arm shoulder link
        np.copyto(self._imu_frames[8], self._left_arm._current_fk[3])
        np.copyto(self._imu_frames[12], self._right_arm._current_fk[3])
        # Arm elbow link
        np.copyto(self._imu_frames[9], self._left_arm._current_fk[5])
        np.copyto(self._imu_frames[13], self._right_arm._current_fk[5])

        q_rot = np.empty((3, 3), dtype=np.float64)
        imu_rotation = np.empty((3, 3), dtype=np.float64)
        pose_tmp = self._pose_tmp
        quaternion_tmp = self._quaternion_tmp

        for i in range(14):
            # Copy the rotation transformation into our temporary 3x3 matrix
            np.copyto(imu_rotation, self._imu_frames[i][0:3, 0:3])

            # Apply the rotation transformation from the current imu frame to the
            # pose calculated by the gyroscope from the module at the index `i`
            # The imu frame is determined by the FK of the robot model at its
            # current positional feedback. Some imu frames are its kinematic chain's
            # base frame, which means that it remains constant regardless of its
            # orientation in space. (TODO: Is this accurate and a useful comment?)
            np.dot(imu_rotation, self._pose_gyros[0:3, i], out=pose_tmp)
            self._pose_gyros[0:3, i] = pose_tmp

            # Get the orientation from the current module
            # If any values are `NaN`, then the module is not capable of
            # sending orientation data, or the module just did not send any data
            # for some reason. If no data was received, write nans in the current
            # column of the roll-pitch-yaw matrix
            np.copyto(quaternion_tmp, orientation[i, 0:4])
            if math_utils.any_nan(quaternion_tmp):
                self._rpy[0:3, i] = np.nan
            else:
                # `quat2rot` is a helper function which
                # converts a quaternion vector into a 3x3 rotation matrix
                math_utils.quat2rot(quaternion_tmp, q_rot)
                # Apply module's orientation (rotation transform) to the module's
                # Rotation matrix transpose is same as inverse,
                # since `q_rot` is an SO(3) matrix
                q_rot = q_rot @ imu_rotation.T
                # `rot2ea` is a helper function which
                # converts a 3x3 rotation matrix into a roll-pitch-yaw Euler angle vector
                math_utils.rot2ea(q_rot, self._rpy[0:3, i])

        # TODO: document this
        self._pose_gyros_mean = np.nanmean(self._pose_gyros[:, self._imu_modules], axis=1)

    def _calculate_lean_angle(self):
        """The CoM of all individual bodies and pose estimate have been updated
        at this point."""
        imu_modules = self._imu_modules

        # {roll,pitch}_angle are in radians here
        roll_angle = np.nanmean(self._rpy[0, imu_modules])
        pitch_angle = np.nanmean(self._rpy[1, imu_modules])

        # Update roll transform (rotation matrix)
        math_utils.rotate_x(roll_angle, output=self._roll_rot)
        # Update pitch transform (rotation matrix)
        math_utils.rotate_y(pitch_angle, output=self._pitch_rot)

        self._roll_angle = degrees(roll_angle)
        self._pitch_angle = degrees(pitch_angle)

        # Rotate by the pitch angle about the Y axis, followed by
        # rotating by the roll angle about the X axis
        np.matmul(self._pitch_rot, self._roll_rot, out=self._T_pose_rot)

        # rad/s
        self._feedback_lean_angle_velocity = self._pose_gyros_mean[1]

        # Find the mean of the two legs' translation vectors at the endeffector
        # and store it in `self._ground_point`
        np.add(self._left_leg.current_tip_fk[0:3, 3],
               self._right_leg.current_tip_fk[0:3, 3],
               out=self._ground_point)
        self._ground_point *= 0.5

        # Get the vector starting from the "ground point" and ending at the
        # position of the current center of mass, then rotate it according to
        # the current pitch angle of Igor, storing the result in `self._line_com`
        np.subtract(self._com, self._ground_point, out=self._line_com)
        np.dot(self._pitch_rot, self._line_com, out=self._line_com)

        # Get the magnitude of the "line CoM" and store it for later use.
        # This is used to scale parameters in the balance controller.
        self._height_com = np.linalg.norm(self._line_com)

        # Using the line center of mass, find the feedback lean angle
        self._feedback_lean_angle = degrees(atan2(self._line_com[0], self._line_com[2]))

        # Update Igor's center of mass by applying the transforms in the following order:
        #  1) pitch rotation
        #  2) roll rotation
        np.dot(self._T_pose_rot, self._com, out=self._com)

        # Copy the pose rotation matrix back into the pose transformation matrix
        self._pose_transform[0:3, 0:3] = self._T_pose_rot

        # Update CoM of legs based on current pose estimate from calculated lean angle
        l_leg_coms = self._left_leg.current_coms
        r_leg_coms = self._right_leg.current_coms
        # Both legs have the same amount of kinematic bodies, so
        # do calculations for both legs in same loop
        for i in range(len(l_leg_coms)):
            leg_com = l_leg_coms[i]
            np.matmul(self._pose_transform, leg_com, out=leg_com)
            leg_com = r_leg_coms[i]
            np.matmul(self._pose_transform, leg_com, out=leg_com)

# ------------------------------------------------
# Actions
# ------------------------------------------------

    def _enter_idle(self):
        group = self._group
        group_command = self._group_command

        from time import sleep

        mio = self._mobile_io
        mio.set_snap(3, 0)
        mio.set_snap(6, 0)
        mio.set_button_output(3, 1)
        mio.set_button_output(4, 0)
        mio.set_axis_value(5, -1)
        mio.set_led_color('blue')

        if self._num_spins > 0:
            # This is not the first time we have been in the idle state. We will clear
            # the state of the body objects before beginning the loop below.
            self._chassis.reset_state()
            self._left_leg.reset_state()
            self._right_leg.reset_state()
            self._left_arm.reset_state()
            self._right_arm.reset_state()
            # Stop commanding while in idle
            group_command.clear()

        self._leave_idle_flag = False
        while not self._leave_idle_flag:
            if self._quit_flag:
                # User requested to quit, so bail out before actually being instructed to go into running mode.
                return

            group_command.led.color = 'magenta'
            group.send_command(group_command)
            sleep(0.1)
            group_command.led.color = 'green'
            group.send_command(group_command)
            sleep(0.1)

        # Let module control the LED color
        group_command.led.color = 'transparent'
        group.send_command(group_command)

        mio = self._mobile_io
        mio.set_button_output(3, 0)
        mio.set_button_output(4, 1)
        mio.set_axis_value(5, -1)
        mio.set_led_color('green')


    def _soft_startup(self):
        """Performs the startup phase, which takes about 3 seconds."""
        if self._quit_flag:
            # Can happen if user was in running mode, re-entered idle, then requested to quit during idle
            return

        l_arm = self._left_arm
        r_arm = self._right_arm
        l_leg = self._left_leg
        r_leg = self._right_leg

        group = self._group
        group.get_next_feedback(reuse_fbk=self._group_feedback)

        self._time_last[:] = self._group_feedback.receive_time

        l_arm_t = l_arm.create_home_trajectory()
        r_arm_t = r_arm.create_home_trajectory()
        l_leg_t = l_leg.create_home_trajectory()
        r_leg_t = r_leg.create_home_trajectory()

        start_time = time()
        t = 0.0

        t_pose = self._pose_transform
        gravity = -1.0 * t_pose[2, 0:3]

        # Update time for chassis
        self._chassis.update_time()

        left_knee = 0.0
        right_knee = 0.0
        soft_start_scale = 1.0 / 3.0

        while t < 3.0:
            # Limit commands initially
            soft_start = min(t * soft_start_scale, 1.0)

            grav_comp_efforts = l_arm.get_grav_comp_efforts(gravity)
            pos, vel, _ = l_arm_t.get_state(t)
            self.left_arm._cmd.position = pos
            self.left_arm._cmd.velocity = vel
            np.multiply(grav_comp_efforts, soft_start, out=self.left_arm._cmd.effort)

            grav_comp_efforts = r_arm.get_grav_comp_efforts(gravity)
            pos, vel, _ = r_arm_t.get_state(t)
            self.right_arm._cmd.position = pos
            self.right_arm._cmd.velocity = vel
            np.multiply(grav_comp_efforts, soft_start, out=self.right_arm._cmd.effort)

            pos, vel, _ = l_leg_t.get_state(t)
            self.left_leg._cmd.position = pos
            self.left_leg._cmd.velocity = vel
            left_knee = pos[1]

            pos, vel, _ = r_leg_t.get_state(t)
            self.right_leg._cmd.position = pos
            self.right_leg._cmd.velocity = vel
            right_knee = pos[1]

            # Scale effort
            group.send_command(self._group_command)
            group.get_next_feedback(reuse_fbk=self._group_feedback)
            t = time() - start_time

        self._time_last[:] = self._group_feedback.receive_time
        self._left_leg._knee_angle = left_knee
        self._right_leg._knee_angle = -right_knee

    def _spin_once(self, bc):
        """
        :param bc: Denotes whether the balance controller is enabled at this time
        :type bc:  bool
        """
        self._group.get_next_feedback(reuse_fbk=self._group_feedback)

        rel_time = time() - self._start_time
        soft_start = min(rel_time, 1.0)
        rx_time = self._group_feedback.receive_time

        if self._config.is_imitation:
            dt = 0.01
        else:
            np.subtract(rx_time, self._time_last, out=self._diff_time)
            dt = np.mean(self._diff_time)

        np.copyto(self._time_last, rx_time)

        # TODO: cache these too
        gyro = self._group_feedback.gyro
        orientation = self._group_feedback.orientation

        if self._config.is_imitation:
            gyro[:, :] = 0.33
            orientation[:, 0] = 0.25
            orientation[:, 1] = 1.0
            orientation[:, 2:4] = 0.0

        self._update_com()
        self._update_pose_estimate(gyro, orientation)
        self._calculate_lean_angle()

        # ----------------------------
        # Value Critical section begin
        self._value_lock.acquire()

        with self._value_lock:
            # For now, we only use the left arm velocity values.
            # This is because we send the same values to both the left and right arm,
            # and also because the chassis trajectory only has 3 command entries for arm velocity values

            # The knee velocities are commanded by the user (joystick `OPTIONS`), and the event callback
            # sets the velocity on the Leg objects in response to user interaction
            user_commanded_knee_velocity = self._left_leg.user_commanded_knee_velocity
            user_commanded_grip_velocity = self._left_arm.user_commanded_grip_velocity

            self._chassis.update_trajectory(user_commanded_knee_velocity, user_commanded_grip_velocity)
            self._chassis.integrate_step(dt)

            calculated_knee_velocity = self._chassis.calculated_knee_velocity
            calculated_grip_velocity = self._chassis.calculated_grip_velocity

            self._left_leg.integrate_step(dt, calculated_knee_velocity)
            self._right_leg.integrate_step(dt, calculated_knee_velocity)

            self._left_arm.integrate_step(dt, calculated_grip_velocity)
            self._right_arm.integrate_step(dt, calculated_grip_velocity)

            self._chassis.update_velocity_controller(dt, self._group_feedback.velocity, self._wheel_radius,
                                                     self._height_com, self._feedback_lean_angle_velocity,
                                                     self._mass, self._feedback_lean_angle)

            #bc = False
            if bc:  # bc
                leanP = Igor.Lean_P
                leanI = Igor.Lean_I
                leanD = Igor.Lean_D

                l_wheel = self._group_command[0]
                r_wheel = self._group_command[1]

                print(f'LeanAngle: {self._feedback_lean_angle}  |  LeanVel: {self._feedback_lean_angle_velocity}')

                p_effort = (leanP * self._chassis.lean_angle_error) +\
                           (leanI * self._chassis.lean_angle_error_cumulative) +\
                           (leanD * self._feedback_lean_angle_velocity)
                effort = p_effort * soft_start
                l_wheel.effort = effort
                r_wheel.effort = -effort
            else:
                self._group_command[0].effort = None
                self._group_command[1].effort = None

            # --------------------------------
            # Wheel Commands
            max_velocity = 10.0
            l_wheel_vel = self._chassis.calculated_yaw_velocity + self._chassis.velocity_feedforward
            r_wheel_vel = self._chassis.calculated_yaw_velocity - self._chassis.velocity_feedforward
            # Limit velocities
            l_wheel_vel = min(max(l_wheel_vel, -max_velocity), max_velocity)
            r_wheel_vel = min(max(r_wheel_vel, -max_velocity), max_velocity)
            self._group_command[0].velocity = l_wheel_vel
            self._group_command[1].velocity = r_wheel_vel

            # ------------
            # Leg Commands

            self.left_leg.update_command(self._roll_angle, soft_start)
            self.right_leg.update_command(self._roll_angle, soft_start)

            # ------------
            # Arm Commands

            self.left_arm.update_command(self._pose_transform, soft_start)
            self.right_arm.update_command(self._pose_transform, soft_start)

        # --------------------------
        # Value Critical section end

        self._group.send_command(self._group_command)

# ------------------------------------------------
# Lifecycle functions
# ------------------------------------------------

    def _on_stop(self):
        for entry in self._on_stop_callbacks:
            entry()

    def _stop(self):
        """Stop running Igor.

        This happens once the user requests the demo to stop, or when
        the running application begins to shut down.
        """
        print('Shutting down Igor...')
        duration = self._stop_time - self._start_time
        tics = float(self._num_spins)
        avg_frequency = tics / duration
        #print('Ran for: {0} seconds.'.format(duration))
        #print('Average processing frequency: {0} Hz'.format(avg_frequency))

        # Set the LED color strategy back to the default
        self._group_command.clear()
        self._group_command.led.color = 'transparent'
        self._group.send_command_with_acknowledgement(self._group_command)

        self._on_stop()

    def _start(self):
        """Main processing method.

        This runs on a background thread.
        """
        first_run = True
        while True:
            self._enter_idle()
            self._soft_startup()

            # Delay registering event handlers until now, so Igor
            # can start up without being interrupted by user commands.
            # View this function in `event_handlers.py` to see
            # all of the joystick event handlers registered
            if first_run:
                try:
                    register_igor_event_handlers(self)
                except Exception as e:
                    print('Caught exception while registering event handlers:\n{0}'.format(e))
                self._start_time = time()
                first_run = False

            self._state_lock.acquire()
            while self._should_continue():
                bc = self._balance_controller_enabled
                self._state_lock.release()
                self._spin_once(bc)
                self._num_spins = self._num_spins + 1
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

# ------------------------------------------------
# Initialization functions
# ------------------------------------------------

    def __init__(self, has_camera=False, config: Igor2Config = None):
        if config is None:
            self._config = Igor2Config()
        elif type(config) is not Igor2Config:
            raise TypeError
        else:
            self._config = config

        if has_camera:
            num_dofs = 15
        else:
            num_dofs = 14

        # The joystick interface
        self._mobile_io = None

        # ------------
        # Group fields
        # These group message objects are only used by the proc thread
        # They are NOT thread safe.
        self._group = None
        self._group_command = None
        self._group_feedback = None
        self._group_info = None

        self._proc_thread = None

        self._on_stop_callbacks = list()

        # ----------------
        # Parameter fields
        self._has_camera = has_camera
        self._joy_dead_zone = 0.06
        self._wheel_radius = 0.100
        self._wheel_base = 0.43

        # ------------
        # State fields
        from threading import Lock
        self._state_lock = Lock()
        self._started = False
        self._balance_controller_enabled = True
        self._quit_flag = False
        self._transition_to_idle_possible = False
        self._leave_idle_flag = False
        self._time_last = np.empty(num_dofs, dtype=np.float64)
        self._diff_time = np.empty(num_dofs, dtype=np.float64)
        value_lock = Lock()
        self._value_lock = value_lock
        self._start_time = -1.0
        self._stop_time = -1.0
        self._num_spins = 0

        # ---------------------
        # Kinematic body fields
        chassis = Chassis(value_lock)
        # path to 'hrdf' folder
        hrdf_dir = os.path.join(os.path.dirname(__file__), '..', 'hrdf')

        model = hebi.robot_model.import_from_hrdf(os.path.join(hrdf_dir, 'tgorLeg.hrdf'))
        base_frame = np.eye(4, dtype=np.float64)
        base_frame[:3, 3] = [0.0, 0.15, 0.0]
        base_frame[:3, :3] = math_utils.rotate_x(np.pi * -0.5)
        model.base_frame = base_frame
        l_leg = Leg(value_lock, 'Left', [2, 3], robot_model=model)

        model = hebi.robot_model.import_from_hrdf(os.path.join(hrdf_dir, 'tgorLeg.hrdf'))
        base_frame = np.eye(4, dtype=np.float64)
        base_frame[:3, 3] = [0.0, -0.15, 0.0]
        base_frame[:3, :3] = math_utils.rotate_x(np.pi * 0.5)
        model.base_frame = base_frame
        r_leg = Leg(value_lock, 'Right', [4, 5], robot_model=model)

        model = hebi.robot_model.import_from_hrdf(os.path.join(hrdf_dir, 'tgorArm-Left.hrdf'))
        base_frame = np.eye(4, dtype=np.float64)
        base_frame[0:3, 3] = [0.0, 0.10, 0.20]
        model.base_frame = base_frame
        l_arm = Arm(value_lock, 'Left', [6, 7, 8, 9], robot_model=model)

        model = hebi.robot_model.import_from_hrdf(os.path.join(hrdf_dir, 'tgorArm-Right.hrdf'))
        base_frame = np.eye(4, dtype=np.float64)
        base_frame[0:3, 3] = [0.0, -0.10, 0.20]
        model.base_frame = base_frame
        r_arm = Arm(value_lock, 'Right', [10, 11, 12, 13], robot_model=model)

        self._chassis = chassis
        self._left_leg = l_leg
        self._right_leg = r_leg
        self._left_arm = l_arm
        self._right_arm = r_arm

        # ---------------------------
        # Cached fields for body mass
        self._mass = l_leg.mass + r_leg.mass + l_arm.mass + r_arm.mass + chassis.mass

        self._masses = np.array([l_leg.mass, r_leg.mass,
                                 l_arm.mass, r_arm.mass, chassis.mass],
                                dtype=np.float64)

        # --------------------------------------------
        # Cached fields for center of mass calculation
        self._com = np.empty(3, dtype=np.float64)
        self._coms = np.zeros((3, 5), dtype=np.float64)
        # This CoM is constant
        self._coms[0:3, 4] = chassis.com

        # ----------------------------------
        # Cached fields for pose estimation
        self._imu_frames = [np.identity(4, dtype=np.float64) for i in range(15)]

        # These frames are constant
        np.copyto(self._imu_frames[2], l_leg._kin.base_frame)
        np.copyto(self._imu_frames[4], r_leg._kin.base_frame)
        np.copyto(self._imu_frames[6], l_arm._kin.base_frame)
        np.copyto(self._imu_frames[10], r_arm._kin.base_frame)

        # All of the leg modules, plus the wheel modules
        self._imu_modules = [0, 1, 2, 3, 4, 5]

        self._pose_gyros = np.zeros((3, num_dofs), dtype=np.float64)
        self._pose_gyros_mean = None

        # Roll-Pitch-Yaw
        self._rpy = np.zeros((3, num_dofs), dtype=np.float64)

        self._pose_transform = np.identity(4, dtype=np.float64)

        # ----------------------------------------
        # Cached fields for lean angle calculation
        self._roll_angle = 0.0
        self._roll_rot = np.zeros((3, 3), dtype=np.float64)
        self._pitch_angle = 0.0
        self._pitch_rot = np.zeros((3, 3), dtype=np.float64)
        self._feedback_lean_angle_velocity = 0.0
        # These are used for chassis velocity controller
        self._height_com = 0.0
        self._line_com = np.zeros(3, dtype=np.float64)
        self._ground_point = np.zeros(3, dtype=np.float64)
        self._feedback_lean_angle = 0.0

        self._pose_tmp = np.zeros(3, dtype=np.float64)
        self._quaternion_tmp = np.zeros(4, dtype=np.float64)
        self._T_pose_rot = np.zeros((3, 3), dtype=np.float64)

# ------------------------------------------------
# Public Interface
# ------------------------------------------------

    def start(self):
        """Start running Igor with the provided configuration.

        This can safely be called multiple times but will do nothing
        after the first invocation. All processing will be done on a
        background thread which is spawned in this method.
        """
        with self._state_lock:
            if self._started:
                self._state_lock.release()
                return

            group = create_group(self._config, self._has_camera)
            group.command_lifetime = 500
            group.feedback_frequency = 100.0

            self._group = group
            self._group_command = hebi.GroupCommand(group.size)
            self._group_feedback = hebi.GroupFeedback(group.size)
            self._group_info = hebi.GroupInfo(group.size)

            self._left_leg.set_message_views(self._group_command, self._group_feedback)
            self._right_leg.set_message_views(self._group_command, self._group_feedback)
            self._left_arm.set_message_views(self._group_command, self._group_feedback)
            self._right_arm.set_message_views(self._group_command, self._group_feedback)

            setup_controller = self._config.setup_controller

            while True:
                try:
                    mio = setup_controller()
                    if mio is None:
                        raise RuntimeError
                    self._mobile_io = mio
                    break
                except:
                    pass

            def idle_to_running(fbk: 'GroupFeedback'):
                data_getter = label_to_pin_map[self._config.controller_mapping.exit_idle_mode]
 
                if data_getter(fbk) == 1:
                    self._leave_idle_flag = True

            self._mobile_io._group.add_feedback_handler(idle_to_running)

            load_gains(self)

            from threading import Condition, Lock, Thread
            start_condition = Condition(Lock())

            def start_routine(start_condition):
                start_condition.acquire()

                # Calling thread holds `__state_lock` AND
                # is also blocked until we call `start_condition.notify_all()`
                # So we can safely modify this here.
                self._started = True

                # Allow the calling thread to continue
                start_condition.notify_all()
                start_condition.release()

                self._start()

            self._proc_thread = Thread(target=start_routine,
                                       name='Igor II Controller',
                                       args=(start_condition,))

            # We will have started the thread before returning,
            # but make sure the function has begun running before
            # we release the `_state_lock` and return
            with start_condition:
                self._proc_thread.start()
                start_condition.wait()


    def request_stop(self):
        """Send a request to stop the demo. If the demo has not been started,
        this method raises an exception.

        :raises RuntimeError: if `start()` has not been called
        """
        self._state_lock.acquire()
        self._ensure_started()
        self._quit_flag = True
        self._state_lock.release()

    def allow_transition_to_idle(self, value):
        """Set if the transition back to idle mode is allowed if igor is
        crouched low enough. If the demo has not been started, this method
        raises an exception.

        :raises RuntimeError: if `start()` has not been called
        """
        self._state_lock.acquire()
        self._ensure_started()
        self._transition_to_idle_possible = bool(value)
        self._state_lock.release()

    def set_balance_controller_state(self, state):
        """Set the state of the balance controller. If the demo has not been
        started, this method raises an exception.

        :param state: ``True`` for enabled, ``False`` for disabled
        :type state:  bool

        :raises TypeError:    if `state` is not bool
        :raises RuntimeError: if `start()` has not been called
        """
        if type(state) is not bool:
            raise TypeError
        self._state_lock.acquire()
        self._ensure_started()
        self._balance_controller_enabled = state
        self._state_lock.release()

# ------------------------------------------------
# Properties
# ------------------------------------------------

    @property
    def mobile_io(self):
        return self._mobile_io

    @property
    def started(self):
        """
        :rtype: bool
        """
        self._state_lock.acquire()
        val = self._started
        self._state_lock.release()
        return val

    @property
    def wheel_radius(self):
        """
        The radius (in meters) of the wheels on this Igor instance
        :rtype: float
        """
        return self._wheel_radius

    @property
    def wheel_base(self):
        """
        :rtype: float
        """
        return self._wheel_base

    @property
    def mass(self):
        """
        The total weight (in kilograms) of this Igor instance
        :rtype: float
        """
        return self._mass

    @property
    def left_arm(self):
        """
        :rtype: Arm
        """
        return self._left_arm

    @property
    def right_arm(self):
        """
        :rtype: Arm
        """
        return self._right_arm

    @property
    def left_leg(self):
        """
        :rtype: Leg
        """
        return self._left_leg

    @property
    def right_leg(self):
        """
        :rtype: Leg
        """
        return self._right_leg

    @property
    def chassis(self):
        """
        :rtype: Chassis
        """
        return self._chassis
