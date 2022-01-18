import sys
import os
from collections import namedtuple
from time import time, sleep
from enum import Enum, auto

import numpy as np

import hebi
from hebi.util import create_mobile_io

from tready_utils import set_mobile_io_instructions, setup_base


class TreadedBase:
    # FRAME CONVENTION:
    # ORIGIN = MID-POINT BETWEEN THE WHEELS
    # +X-AXIS = FORWARD
    # +Y-AXIS = LEFT
    # +Z-AXIS = UP

    #   Left  |   Right
    #   1     |    2
    #         |
    #         |
    #   3     |    4

    WHEEL_DIAMETER = 0.105
    WHEEL_BASE = 0.400

    WHEEL_RADIUS = WHEEL_DIAMETER / 2

    def __init__(self, group, chassis_ramp_time, flipper_ramp_time):
        self.group = group
        self.fbk = hebi.GroupFeedback(group.size)
        self.wheel_fbk = self.fbk.create_view([0, 1, 2, 3])
        self.flipper_fbk = self.fbk.create_view([4, 5, 6, 7])
        self.cmd = hebi.GroupCommand(group.size)
        self.wheel_cmd = self.cmd.create_view([0, 1, 2, 3])
        self.flipper_cmd = self.cmd.create_view([4, 5, 6, 7])

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.group.get_next_feedback(reuse_fbk=self.fbk)

        self.cmd.position = self.fbk.position
        self.t_prev = time()
        self._aligned_flipper_mode = False
        self._is_aligning = False

        self.chassis_traj = None
        self.flipper_traj = None

    @property
    def has_active_trajectory(self):
        if self.chassis_traj is not None and self.t_prev < self.chassis_traj.end_time:
            return True
        if self.flipper_traj is not None and self.t_prev < self.flipper_traj.end_time:
            return True
        return False

    @property
    def is_aligning(self):
        active_align = self._is_aligning and self.has_active_trajectory
        return active_align and not self.flippers_aligned

    @property
    def aligned_flipper_mode(self):
        return self._aligned_flipper_mode

    @aligned_flipper_mode.setter
    def aligned_flipper_mode(self, value):
        if value == self.aligned_flipper_mode:
            return
        elif value:
            self.align_flippers()
        else:
            self.unlock_flippers()

    @property
    def wheel_to_chassis_vel(self):
        wr = self.WHEEL_RADIUS / (self.WHEEL_BASE / 2)
        return np.array([
            [self.WHEEL_RADIUS, -self.WHEEL_RADIUS, self.WHEEL_RADIUS, -self.WHEEL_RADIUS],
            [0, 0, 0, 0],
            [wr, wr, wr, wr]
        ])

    @property
    def chassis_to_wheel_vel(self):
        return np.array([
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
        ])

    @property
    def flipper_aligned_front(self):
        return np.abs(self.flipper_fbk.position_command[0] + self.flipper_fbk.position_command[1]) < 0.01

    @property
    def flipper_aligned_back(self):
        return np.abs(self.flipper_fbk.position_command[2] + self.flipper_fbk.position_command[3]) < 0.01

    @property
    def flippers_aligned(self):
        return self.flipper_aligned_front and self.flipper_aligned_back

    @property
    def aligned_flipper_position(self):
        front_mean = np.mean([self.flipper_fbk.position_command[0], -1 * self.flipper_fbk.position_command[1]])
        back_mean = np.mean([self.flipper_fbk.position_command[2], -1 * self.flipper_fbk.position_command[3]])
        return np.array([front_mean, -front_mean, back_mean, -back_mean], dtype=np.float64)

    def update(self, t_now):
        dt = t_now - self.t_prev
        self.group.get_next_feedback(reuse_fbk=self.fbk)

        if self.flipper_traj is None and self.chassis_traj is None:
            print("No trajectories, zeroing velocity")
            self.cmd.velocity = 0.0
        else:
            if self.chassis_traj is not None:
                # chassis update
                t = min(t_now, self.chassis_traj.end_time)
                [_, vel, _] = self.chassis_traj.get_state(t)
                self.wheel_cmd.velocity = self.chassis_to_wheel_vel @ vel
                self.wheel_cmd.position += self.wheel_cmd.velocity * dt

            if self.flipper_traj is not None:
                # flipper update
                t = min(t_now, self.flipper_traj.end_time)
                [pos, vel, _] = self.flipper_traj.get_state(t)

                self.flipper_cmd.velocity = vel
                if self.aligned_flipper_mode:
                    if not self.flippers_aligned:
                        self.flipper_cmd.position = pos
                    else:
                        self.flipper_cmd.position = self.aligned_flipper_position + vel * dt
                else:
                    self.flipper_cmd.position += vel * dt

        self.t_prev = t_now

    def send(self):
        self.group.send_command(self.cmd)

    def set_flipper_trajectory(self, t_now, ramp_time, p=None, v=None):
        # This is set to true after this call, if the trajectory is an aligning one
        # Otherwise we want it set to False, so clear it now
        self._is_aligning = False
        times = [t_now, t_now + ramp_time]
        positions = np.empty((4, 2))
        velocities = np.empty((4, 2))
        accelerations = np.empty((4, 2))

        if self.flipper_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            positions[:, 0], velocities[:, 0], accelerations[:, 0] = self.flipper_traj.get_state(t)
        else:
            positions[:, 0] = self.flipper_fbk.position
            velocities[:, 0] = self.flipper_fbk.velocity
            accelerations[:, 0] = self.flipper_fbk.effort_command

        positions[:, 1] = np.nan if p is None else p
        velocities[:, 1] = 0.0 if v is None else v
        accelerations[:, 1] = 0.0

        self.flipper_traj = hebi.trajectory.create_trajectory(times, positions, velocities, accelerations)

    def set_chassis_vel_trajectory(self, t_now, ramp_time, v):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((3, 2))
        velocities = np.empty((3, 2))
        efforts = np.empty((3, 2))

        if self.chassis_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            positions[:, 0], velocities[:, 0], efforts[:, 0] = self.chassis_traj.get_state(t)
        else:
            positions[:, 0] = 0.0
            velocities[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.velocity
            efforts[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.effort

        positions[:, 1] = np.nan
        velocities[:, 1] = v
        efforts[:, 1] = 0.0

        self.chassis_traj = hebi.trajectory.create_trajectory(times, positions, velocities, efforts)

    def align_flippers(self, t_now=None):
        t_now = t_now or self.t_prev
        print("FLIPPER ALIGNMENT ON")
        self._aligned_flipper_mode = True
        self.chassis_traj = None
        if not self.flippers_aligned:
            print("Setting aligning trajectory")
            self.set_flipper_trajectory(t_now, 3.0, p=self.aligned_flipper_position)
            self._is_aligning = True

    def unlock_flippers(self):
        print("FLIPPER ALIGNMENT OFF")
        self._aligned_flipper_mode = False

    def set_color(self, color):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.clear()
        self.group.send_command(color_cmd)


class DemoState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    STOPPED = auto()
    EXIT = auto()


ChassisVelocity = namedtuple('ChassisVelocity', ['x', 'rz'])
TreadyInputs = namedtuple('TreadyInputs', ['base_motion', 'flippers', 'align_flippers'])
DemoInputs = namedtuple('DemoInputs', ['should_exit', 'should_reset', 'chassis'])


class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, mobile_io, base: TreadedBase):
        self.state = DemoState(DemoState.STARTUP)
        self.mobile_io = mobile_io
        self.base = base

        self.SPEED_MAX_LIN = 0.15  # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / base.WHEEL_BASE  # rad/s

    def start_logging(self):
        self.base.group.start_log("logs", mkdirs=True)

    def cycle_log(self):
        self.base.group.stop_log()
        self.base.group.start_log("logs", mkdirs=True)

    def compute_velocities(self, chassis_inputs: ChassisVelocity):
        # Flipper Control
        [flip1, flip2, flip3, flip4] = chassis_inputs.flippers

        if chassis_inputs.align_flippers:
            f_vel1 = max(abs(flip1), abs(flip2)) * np.sign(flip1 + flip2) * self.FLIPPER_VEL_SCALE
            f_vel2 = -f_vel1
            f_vel3 = max(abs(flip3), abs(flip4)) * np.sign(flip3 + flip4) * self.FLIPPER_VEL_SCALE
            f_vel4 = -f_vel3

        else:
            f_vel1 = flip1 * self.FLIPPER_VEL_SCALE
            f_vel2 = -1 * flip2 * self.FLIPPER_VEL_SCALE
            f_vel3 = flip3 * self.FLIPPER_VEL_SCALE
            f_vel4 = -1 * flip4 * self.FLIPPER_VEL_SCALE

        flipper_vels = [f_vel1, f_vel2, f_vel3, f_vel4]

        # Mobile Base Control

        vel_x = self.SPEED_MAX_LIN * chassis_inputs.base_motion.x
        vel_y = 0
        vel_rot = self.SPEED_MAX_ROT * chassis_inputs.base_motion.rz

        chassis_vels = [vel_x, vel_y, vel_rot]

        return chassis_vels, flipper_vels

    def update(self, t_now: float, demo_input=None):
        self.base.update(t_now)
        self.base.send()

        if self.state is DemoState.EXIT:
            return False

        if demo_input is None:
            if t_now - self.mobile_last_fbk_t > 1.0:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, DemoState.STOPPED)
            return True
        else:
            self.mobile_last_fbk_t = t_now

            if demo_input.should_exit:
                self.transition_to(t_now, DemoState.EXIT)
            elif demo_input.should_reset:
                self.transition_to(t_now, DemoState.HOMING)

            if self.state is DemoState.STOPPED:
                self.mobile_last_fbk_t = t_now
                self.transition_to(t_now, DemoState.TELEOP)
                return True

            elif self.state is DemoState.HOMING:
                base_complete = not self.base.has_active_trajectory
                if base_complete:
                    self.transition_to(t_now, DemoState.TELEOP)
                return True

            elif self.state is DemoState.TELEOP:
                desired_flipper_mode = demo_input.chassis.align_flippers
                if self.base.aligned_flipper_mode != desired_flipper_mode:
                    self.base.aligned_flipper_mode = desired_flipper_mode
                elif self.base.is_aligning:
                    # ignore new base commands while flippers are aligning
                    pass
                else:
                    chassis_vels, flipper_vels = self.compute_velocities(demo_input.chassis)
                    self.base.set_chassis_vel_trajectory(t_now, self.base.chassis_ramp_time, chassis_vels)
                    self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)

                return True

            elif self.state is DemoState.STARTUP:
                self.transition_to(t_now, DemoState.HOMING)
                return True

    def transition_to(self, t_now, state):
        # self transitions are noop
        if state == self.state:
            return

        if state is DemoState.HOMING:
            print("TRANSITIONING TO HOMING")
            self.base.set_color('magenta')
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(self.mobile_io, msg)

            # build trajectory
            flipper_home = np.array([-1, 1, 1, -1]) * np.deg2rad(15 + 45)
            self.base.chassis_traj = None
            self.base.set_flipper_trajectory(t_now, 5.0, p=flipper_home)

        elif state is DemoState.TELEOP:
            print("TRANSITIONING TO TELEOP")
            self.base.clear_color()
            msg = ('Robot Ready to Control\n'
                   'B1: Reset\n'
                   'B6: Joined Flipper\n'
                   'B8 - Quit')
            set_mobile_io_instructions(self.mobile_io, msg, color="green")

        elif state is DemoState.STOPPED:
            print("TRANSITIONING TO STOPPED")
            self.base.chassis_traj = None
            self.base.flipper_traj = None
            self.base.set_color('blue')

        elif state is DemoState.EXIT:
            print("TRANSITIONING TO EXIT")
            self.base.set_color("red")

            # unset mobileIO control config
            self.mobile_io.set_button_mode(6, 0)
            self.mobile_io.set_button_output(1, 0)
            self.mobile_io.set_button_output(8, 0)
            set_mobile_io_instructions(self.mobile_io, 'Demo Stopped', color="red")

        self.state = state


def config_mobile_io(m):
    """Sets up mobileIO interface.

    Return a function that parses mobileIO feedback into the format
    expected by the Demo
    """

    # MobileIO Button Config
    reset_pose_btn = 1
    joined_flipper_btn = 6
    quit_btn = 8

    slider_flip1 = 3
    slider_flip2 = 4
    slider_flip3 = 5
    slider_flip4 = 6

    joy_fwd = 2
    joy_rot = 1

    # set mobileIO control config
    m.set_led_color("blue")
    m.set_snap(slider_flip1, 0)
    m.set_snap(slider_flip2, 0)
    m.set_snap(slider_flip3, 0)
    m.set_snap(slider_flip4, 0)

    m.set_button_mode(joined_flipper_btn, 1)

    m.set_button_output(reset_pose_btn, 1)
    m.set_button_output(quit_btn, 1)

    def parse_mobile_io_feedback(m):
        should_exit = m.get_button_state(quit_btn)
        should_reset = m.get_button_state(reset_pose_btn)
        # Chassis Control
        aligned_flipper_mode = m.get_button_state(joined_flipper_btn)
        joy_vel_fwd = m.get_axis_state(joy_fwd)
        joy_vel_rot = m.get_axis_state(joy_rot)

        # Flipper Control
        flip1 = m.get_axis_state(slider_flip1)
        flip2 = m.get_axis_state(slider_flip2)
        flip3 = m.get_axis_state(slider_flip3)
        flip4 = m.get_axis_state(slider_flip4)

        tready_inputs = TreadyInputs(
            ChassisVelocity(joy_vel_fwd, joy_vel_rot),
            [flip1, flip2, flip3, flip4],
            aligned_flipper_mode)

        return DemoInputs(should_exit, should_reset, tready_inputs)

    return parse_mobile_io_feedback


if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    family = "Tready"

    # Base setup
    base = setup_base(lookup, family)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for mobileIO device to come online...')
    m = create_mobile_io(lookup, family, phone_name)
    while m is None:
        m = create_mobile_io(lookup, family, phone_name)
        print("Could not find mobileIO device, waiting...")
        sleep(0.5)

    print("mobileIO device found.")
    input_parser = config_mobile_io(m)

    demo_controller = TreadyControl(m, base)

    #######################
    ## Main Control Loop ##
    #######################

    should_continue = True
    demo_controller.start_logging()
    last_log_start_time = time()
    while should_continue:
        t = time()
        demo_inputs = None
        if m.update(0.0):
            demo_inputs = input_parser(m)

        should_continue = demo_controller.update(t, demo_inputs)
        if t - last_log_start_time > 60:
            demo_controller.cycle_log()
            last_log_start_time = t

    sys.exit(0)
