import re
import sys
import os
from collections import namedtuple
from time import time, sleep
from enum import Enum, auto

import numpy as np

import requests

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Optional, Callable
    import numpy.typing as npt
    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO
layoutsPath = './kits/tready/layouts/'

with open(layoutsPath + 'FourSliderController.json', 'r') as f:
    mainPage = f.read()
with open(layoutsPath + 'TwoSliderController.json', 'r') as f:
    twoPage = f.read()

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

    def __init__(self, group: 'Group', chassis_ramp_time: float, flipper_ramp_time: float):
        self.group = group

        self.fbk = self.group.get_next_feedback()
        while self.fbk == None:
            self.fbk = self.group.get_next_feedback()

        self.wheel_fbk = self.fbk.create_view([0, 1, 2, 3])
        self.flipper_fbk = self.fbk.create_view([4, 5, 6, 7])
        self.cmd = hebi.GroupCommand(group.size)
        self.wheel_cmd = self.cmd.create_view([0, 1, 2, 3])
        self.flipper_cmd = self.cmd.create_view([4, 5, 6, 7])

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.flipper_cmd.position = self.flipper_fbk.position
        self.wheel_cmd.position = np.nan

        self.t_prev: float = time()
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
    def wheel_to_chassis_vel(self) -> 'npt.NDArray[np.float64]':
        wr = self.WHEEL_RADIUS / (self.WHEEL_BASE / 2)
        return np.array([
            [self.WHEEL_RADIUS, -self.WHEEL_RADIUS, self.WHEEL_RADIUS, -self.WHEEL_RADIUS],
            [0, 0, 0, 0],
            [wr, wr, wr, wr]
        ])

    @property
    def chassis_to_wheel_vel(self) -> 'npt.NDArray[np.float64]':
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
    def aligned_flipper_position(self) -> 'npt.NDArray[np.float64]':
        front_mean = np.mean([self.flipper_fbk.position_command[0], -1 * self.flipper_fbk.position_command[1]])
        back_mean = np.mean([self.flipper_fbk.position_command[2], -1 * self.flipper_fbk.position_command[3]])
        return np.array([front_mean, -front_mean, back_mean, -back_mean], dtype=np.float64)

    def update(self, t_now: float):
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

    def set_flipper_trajectory(self, t_now: float, ramp_time: float, p=None, v=None):
        # This is set to true after this call, if the trajectory is an aligning one
        # Otherwise we want it set to False, so clear it now
        self._is_aligning = False
        times = [t_now, t_now + ramp_time]
        positions = np.empty((4, 2), dtype=np.float64)
        velocities = np.empty((4, 2), dtype=np.float64)
        accelerations = np.empty((4, 2), dtype=np.float64)

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

    def set_chassis_vel_trajectory(self, t_now: float, ramp_time: float, v):
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

    def align_flippers(self, t_now: 'Optional[float]'=None):
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

    def set_color(self, color: 'hebi.Color | str'):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.clear()
        self.group.send_command(color_cmd)


class TreadyControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class ChassisVelocity:
    def __init__(self, x: float, rz: float):
        self.x = x
        self.rz = rz


class TreadyInputs:
    def __init__(self, home: bool, base_motion: 'ChassisVelocity', flippers: 'list[float]', align_flippers: bool):
        self.home = home
        self.base_motion = base_motion
        self.flippers = flippers
        self.align_flippers = align_flippers


class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, base: TreadedBase):
        self.state = TreadyControlState.STARTUP
        self.base = base

        self.SPEED_MAX_LIN = 0.15  # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / base.WHEEL_BASE  # rad/s
        self._transition_handlers: 'list[Callable[[TreadyControl, TreadyControlState], None]]' = []

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def start_logging(self):
        self.base.group.start_log("logs", mkdirs=True)

    def cycle_log(self):
        self.base.group.stop_log()
        self.base.group.start_log("logs", mkdirs=True)

    def compute_velocities(self, chassis_inputs: TreadyInputs):
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

        chassis_vels: 'npt.NDArray[np.float64]' = np.array([vel_x, vel_y, vel_rot], dtype=np.float64)

        return chassis_vels, flipper_vels

    def send(self):
        self.base.send()

    def update(self, t_now: float, tready_input: 'Optional[TreadyInputs]'=None):
        self.base.update(t_now)

        if self.state is self.state.EXIT:
            return

        if tready_input is None:
            if t_now - self.mobile_last_fbk_t > 1.0:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return
        else:
            self.mobile_last_fbk_t = t_now

            if tready_input.home:
                self.transition_to(t_now, self.state.HOMING)

            elif self.state is self.state.DISCONNECTED:
                self.mobile_last_fbk_t = t_now
                self.transition_to(t_now, self.state.TELEOP)

            elif self.state is self.state.HOMING:
                base_complete = not self.base.has_active_trajectory
                if base_complete:
                    self.transition_to(t_now, self.state.TELEOP)

            elif self.state is self.state.TELEOP:
                desired_flipper_mode = tready_input.align_flippers
                if self.base.aligned_flipper_mode != desired_flipper_mode:
                    self.base.aligned_flipper_mode = desired_flipper_mode
                elif self.base.is_aligning:
                    # ignore new base commands while flippers are aligning
                    pass
                else:
                    chassis_vels, flipper_vels = self.compute_velocities(tready_input)
                    self.base.set_chassis_vel_trajectory(t_now, self.base.chassis_ramp_time, chassis_vels)
                    self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)

            elif self.state is self.state.STARTUP:
                self.transition_to(t_now, self.state.HOMING)

    def transition_to(self, t_now: float, state: TreadyControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print("TRANSITIONING TO HOMING")
            # build trajectory
            flipper_home = np.array([-1, 1, 1, -1]) * np.deg2rad(15 + 45)
            self.base.set_chassis_vel_trajectory(t_now, 0.25, [0, 0, 0])

            self.base.set_flipper_trajectory(t_now, 5.0, p=flipper_home)
            #print(f'Flipper Home: {flipper_home}')

        elif state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")
            self.base.chassis_traj = None
            self.base.flipper_traj = None

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        for handler in self._transition_handlers:
            handler(self, state)
        self.state = state


def config_mobile_io(m: 'MobileIO'):
    """Sets up mobileIO interface.
    Return a function that parses mobileIO feedback into the format
    expected by the Demo
    """

    # MobileIO Button Config
    reset_pose_btn = 1
    joined_flipper_btn = 2
    quit_btn = 3

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

    def parse_mobile_io_feedback(m: 'MobileIO'):
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

        return TreadyInputs(should_reset,
                            ChassisVelocity(joy_vel_fwd, joy_vel_rot),
                            [flip1, flip2, flip3, flip4],
                            aligned_flipper_mode)

    return parse_mobile_io_feedback


if __name__ == "__main__":
    from .tready_utils import set_mobile_io_instructions, setup_base
    from util.mio_layout import MioLayoutController

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
    mac = ""
    for module in lookup.entrylist:
        print(module)
        if module.family == family and module.name == phone_name:
            mac = module.mac_address
            break;
    mlc = MioLayoutController(m)
    mlc.set_mobile_io_instructions(mainPage)
    m.set_button_label(2, "Join Flippers")
    m.set_button_label(3, "Exit Demo")
    m.set_axis_value(1, "Forward")
    m.set_axis_label(2, "Trun")
    m.set_axis_label(3, "FL")
    m.set_axis_label(4, "FR")
    m.set_axis_label(5, "BL")
    m.set_axis_label(6, "BR")
    
    print("mobileIO device found.")
    input_parser = config_mobile_io(m)

    demo_controller = TreadyControl(base)

    def update_mobile_io(controller: TreadyControl, new_state: TreadyControlState):
        if controller.state == new_state:
            return
        if new_state is controller.state.HOMING:
            controller.base.set_color('magenta')
            print("AAAAAA")
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(m, msg, color="blue")
        elif new_state is controller.state.TELEOP:
            controller.base.clear_color()
            msg = ('Robot Ready to Control\n'
                   'B1: Reset\n'
                   'B6: Joined Flipper\n'
                   'B8 - Quit')
            m.set_button_label(1, "Home")
            m.set_button_label(6, "Join Flippers")
            m.set_button_label(8, "Quit")
            m.clear_text()
            m.add_text(msg)
            set_mobile_io_instructions(m, msg, color="green")

        elif new_state is controller.state.DISCONNECTED:
            controller.base.set_color('blue')

        elif new_state is controller.state.EXIT:
            controller.base.set_color("red")

            # unset mobileIO control config
            m.set_button_mode(6, 0)
            m.set_button_output(1, 0)
            m.set_button_output(8, 0)
            set_mobile_io_instructions(m, 'Demo Stopped', color="red")

    demo_controller._transition_handlers.append(update_mobile_io)

    #######################
    ## Main Control Loop ##
    #######################

    #demo_controller.start_logging()
    #last_log_start_time = time()
    while demo_controller.running:
        t = time()
        demo_inputs = None
        if m.update(0.0):
            demo_inputs = input_parser(m)

        try:
            demo_controller.update(t, demo_inputs)
            demo_controller.send()
        except KeyboardInterrupt:
            demo_controller.transition_to(t, TreadyControlState.EXIT)

        if m.get_button_state(8):
            demo_controller.transition_to(t, TreadyControlState.EXIT)

        #if t - last_log_start_time > 3600:
        #    demo_controller.cycle_log()
        #    last_log_start_time = t


    sys.exit(0)