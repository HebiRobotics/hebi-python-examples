import sys
import os
from collections import namedtuple
from time import time, sleep
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Optional, Callable
    import numpy.typing as npt
    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO


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

    WHEEL_DIAMETER = 0.125 # m
    WHEEL_BASE = 0.285 # m

    WHEEL_RADIUS = WHEEL_DIAMETER / 2

    TORSO_VEL_SCALE = 1 # rad/s
    TORSO_TORQUE_SCALE = 2.5 # Nm
    TORQUE_MODE_MAX = 25 # Nm
    TORQUE_ANGLE_OFFSET = np.pi/4
    FLIPPER_HOME_POS = np.pi/3

    def __init__(self, group: 'Group', chassis_ramp_time: float, flipper_ramp_time: float):
        self.group = group

        fbk = self.group.get_next_feedback()
        while fbk is None:
            fbk = self.group.get_next_feedback()
        self.fbk = fbk

        self.wheel_fbk = self.fbk.create_view([0, 1, 2, 3])
        self.flipper_fbk = self.fbk.create_view([4, 5, 6, 7])
        self.cmd = hebi.GroupCommand(group.size)
        self.wheel_cmd = self.cmd.create_view([0, 1, 2, 3])
        self.flipper_cmd = self.cmd.create_view([4, 5, 6, 7])

        self.flipper_sign = np.array([-1, 1, 1, -1])

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.flipper_cmd.position = self.flipper_fbk.position
        self.wheel_cmd.position = np.nan

        self.t_prev: float = time()
        self._aligned_flipper_mode = False

        self.chassis_traj = None
        self.flipper_traj = None

        self.robot_model = None
    
    @property
    def mstop_pressed(self):
        return any(self.fbk.mstop_state == 0)

    @property
    def has_active_base_trajectory(self):
        if self.chassis_traj is not None and self.t_prev < self.chassis_traj.end_time:
            return True
        return False
    
    @property
    def has_active_flipper_trajectory(self):
        if self.flipper_traj is not None and self.t_prev < self.flipper_traj.end_time:
            return True
        return False

    @property
    def has_active_trajectory(self):
        return self.has_active_base_trajectory or self.has_active_flipper_trajectory
    
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
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
        ])

    @property
    def aligned_flipper_position(self) -> 'npt.NDArray[np.float64]':
        mean_pos = np.mean(np.abs(self.flipper_fbk.position))
        return np.array([-mean_pos, mean_pos, mean_pos, -mean_pos], dtype=np.float64)

    @property
    def flipper_height(self) -> 'npt.NDArray[np.float64]':
        x = np.cos(self.flipper_sign * self.TORQUE_ANGLE_OFFSET + self.flipper_fbk.position)
        return (1 + self.WHEEL_BASE * np.clip(x, 0, 1) / self.WHEEL_RADIUS)
    
    @property
    def pose(self) -> 'npt.NDArray[np.float64]':
        # Use Pose estimate of a single flipper actuator in Tready to get the body Pose estimate
        pos = self.fbk.position
        position = []
        for idx in range(0, 4):
            position.append(pos[idx+4])
            position.append(pos[idx])

        frames = self.robot_model.get_forward_kinematics_mat('com', position)
        track_rot_mat = frames[10, :3, :3]

        quat = self.fbk.orientation[1]
        q_track = np.array([quat[1], quat[2], quat[3], quat[0]])
        rot_mat_tready = R.from_quat(q_track).as_matrix()
        rot_mat_tready = rot_mat_tready @ track_rot_mat.T

        # convert to euler
        rpy = R.from_matrix(rot_mat_tready).as_euler('xyz', degrees=True)
        return rpy
    
    def update_feedback(self):
        self.group.get_next_feedback(reuse_fbk=self.fbk)

    def update(self, t_now: float, get_feedback: bool = True):
        if get_feedback:
            self.group.get_next_feedback(reuse_fbk=self.fbk)

        if self.flipper_traj is None and self.chassis_traj is None:
            # print("No trajectories, zeroing velocity")
            self.cmd.velocity = 0.0
        else:
            if self.chassis_traj is not None:
                # chassis update
                t = min(t_now, self.chassis_traj.end_time)
                [_, vel, _] = self.chassis_traj.get_state(t)

                flipper_height = self.flipper_height
                # Moving average setup below, in an attempt to make Tready less wobbly on tiptoes
                flipper_vels = -1.0 * self.flipper_fbk.velocity_command
                if np.any(np.isnan(flipper_vels)):
                    flipper_vels = self.flipper_fbk.gyro[:, 2]



                self.wheel_cmd.position[:] = np.nan
                self.wheel_cmd.velocity = self.chassis_to_wheel_vel @ vel + flipper_vels * flipper_height
                for i in range(len(self.wheel_cmd.effort)):
                    if flipper_height[i] > 1 - 1e-12:
                        self.wheel_cmd.effort[i] = np.nan
                    else:
                        self.wheel_cmd.effort[i] = self.flipper_sign[i] * np.tanh(-flipper_height[i] * (1 + self.WHEEL_BASE / self.WHEEL_RADIUS))

            if self.flipper_traj is not None:
                # flipper update
                t = min(t_now, self.flipper_traj.end_time)
                [_, vel, _] = self.flipper_traj.get_state(t)

                self.flipper_cmd.velocity = vel
                self.flipper_cmd.position += vel * (t_now - self.t_prev)

        self.t_prev = t_now

    def send(self):
        self.group.send_command(self.cmd)
    
    def set_flipper_cmd(self, p=None, v=None, e=None):
        if p is not None:
            self.flipper_cmd.position = p
        if v is not None:
            self.flipper_cmd.velocity = v
        if e is not None:
            self.flipper_cmd.effort = e
    
    def set_chassis_cmd(self, p=None, v=None, e=None):
        if p is not None:
            self.wheel_cmd.position = p
        if v is not None:
            self.wheel_cmd.velocity = v
        if e is not None:
            self.wheel_cmd.effort = e

    def set_flipper_trajectory(self, t_now: float, ramp_time: float, p=None, v=None):
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
            t = min(t_now, self.chassis_traj.end_time)
            positions[:, 0], velocities[:, 0], efforts[:, 0] = self.chassis_traj.get_state(t)
        else:
            positions[:, 0] = 0.0
            velocities[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.velocity
            efforts[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.effort

        positions[:, 1] = np.nan
        velocities[:, 1] = v
        efforts[:, 1] = 0.0

        self.chassis_traj = hebi.trajectory.create_trajectory(times, positions, velocities, efforts)

    def home(self, t_now: float):
        flipper_home = self.flipper_sign * self.FLIPPER_HOME_POS
        self.set_chassis_vel_trajectory(t_now, 0.25, [0, 0, 0])
        self.set_flipper_trajectory(t_now, 3.0, p=flipper_home)

    def align_flippers(self, t_now: float):
        self.set_flipper_trajectory(t_now, 3.0, p=self.aligned_flipper_position)
    
    def set_robot_model(self, hrdf_file: str):
        self.robot_model = hebi.robot_model.import_from_hrdf(hrdf_file)

    def set_color(self, color: 'hebi.Color | str'):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = hebi.Color(0, 0, 0, 0)
        self.group.send_command(color_cmd)


class TreadyControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    ALIGNING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EMERGENCY_STOP = auto()
    EXIT = auto()


class ChassisVelocity:
    def __init__(self, x: float = 0, rz: float = 0):
        self.x = x
        self.rz = rz
    
    def __repr__(self) -> str:
        return f'ChassisVelocity(x={self.x}, z={self.z}, rz={self.rz})'

class TreadyInputs:
    def __init__(self, home: bool = False, base_motion: 'ChassisVelocity' = ChassisVelocity(), flippers: 'list[float]' = [0, 0, 0, 0], align_flippers: bool = False, torque_mode: bool = False, torque_toggle: bool = False):
        self.home = home
        self.base_motion = base_motion
        self.flippers = flippers
        self.align_flippers = align_flippers
        self.torque_mode = torque_mode
        self.torque_toggle = torque_toggle
    
    def __repr__(self) -> str:
        return f'TreadyInputs(home={self.home}, base_motion={self.base_motion}, flippers={self.flippers}, align_flippers={self.align_flippers}, torque_mode={self.torque_mode}, torque_toggle={self.torque_toggle})'


class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, base: TreadedBase):
        self.namespace = ''
        
        self.state = TreadyControlState.STARTUP
        self.base = base

        self.SPEED_MAX_LIN = 0.25  # m/s, for tracks driven by R8-9+
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / (base.WHEEL_BASE / 2) # rad/s
        self._transition_handlers: 'list[Callable[[TreadyControl, TreadyControlState], None]]' = []
        self._update_handlers: 'list[Callable[[TreadyControl], None]]' = []

        # Variable for torque mode update handler
        self.torque_labels = None

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def start_logging(self):
        self.base.group.start_log("logs", mkdirs=True)

    def cycle_log(self):
        self.base.group.stop_log()
        self.base.group.start_log("logs", mkdirs=True)

    def send(self):
        self.base.send()

    def update(self, t_now: float, tready_input: 'Optional[TreadyInputs]'=None):
        self.base.update_feedback()

        if self.state is self.state.EXIT:
            return
        
        if self.base.mstop_pressed and self.state is not self.state.EMERGENCY_STOP:
            self.transition_to(t_now, self.state.EMERGENCY_STOP)
            return

        if tready_input is None and self.state is not self.state.DISCONNECTED and self.state is not self.state.EMERGENCY_STOP:
            if t_now - self.last_cmd_t > 1.0:
                print(self.namespace + "mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return
        
        # Reset the timeout
        self.last_cmd_t = t_now

        if self.state is self.state.EMERGENCY_STOP:
            if not self.base.mstop_pressed:
                print(self.namespace + "Emergency Stop Released")
                self.transition_to(t_now, self.state.TELEOP)
        
        # Transition to teleop if mobileIO is reconnected
        elif self.state is self.state.DISCONNECTED:
            self.last_cmd_t = t_now
            print(self.namespace + 'Controller reconnected, demo continued.')
            self.transition_to(t_now, self.state.TELEOP)
        
        # After startup, transition to homing
        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

        # If homing/aligning is complete, transition to teleop
        elif self.state is self.state.HOMING or self.state is self.state.ALIGNING:
            if not self.base.has_active_flipper_trajectory:
                self.transition_to(t_now, self.state.TELEOP)

        # Teleop mode
        elif self.state is self.state.TELEOP:
            # Check for home button
            if tready_input.home:
                if tready_input.torque_mode:
                    print(self.namespace + "Cannot home in torque mode")
                    return
                self.transition_to(t_now, self.state.HOMING)
            # Check for flipper alignment
            elif tready_input.align_flippers:
                if tready_input.torque_mode:
                    print(self.namespace + "Cannot align flippers in torque mode")
                    return
                self.transition_to(t_now, self.state.ALIGNING)
            else:
                if tready_input.torque_mode:
                    torque_max = self.base.TORQUE_MODE_MAX * ((tready_input.flippers[0] + 1) / 2)
                    torque_angle = (1 + tready_input.flippers[1]) * self.base.TORQUE_ANGLE_OFFSET
                    roll_angle, pitch_angle, _ = self.base.pose

                    roll_adjust = (tready_input.flippers[2] + 1) / 2
                    pitch_adjust = (tready_input.flippers[3] + 1) / 2

                    if roll_angle > 0:
                        roll_torque = np.array([0, -1, 0, 1]) * roll_angle * self.base.TORSO_TORQUE_SCALE * roll_adjust
                    else:
                        roll_torque = np.array([-1, 0, 1, 0]) * roll_angle * self.base.TORSO_TORQUE_SCALE * roll_adjust
                    
                    if pitch_angle > 0:
                        pitch_torque = np.array([1, -1, 0, 0]) * pitch_angle * self.base.TORSO_TORQUE_SCALE * pitch_adjust
                    else:
                        pitch_torque = np.array([0, 0, 1, -1]) * pitch_angle * self.base.TORSO_TORQUE_SCALE * pitch_adjust
                    
                    level_torque = roll_torque + pitch_torque
                    flipper_efforts = -np.tanh(self.base.flipper_fbk.position - self.base.flipper_sign * torque_angle) * torque_max + level_torque

                    self.base.flipper_traj = None
                    self.base.set_flipper_cmd(p=np.ones(4) * np.nan, v = np.ones(4) * np.nan, e=flipper_efforts)

                    self.torque_labels = [
                        f"Max\nEff:\n{np.round(torque_max, 2)}",
                        f"Torque\nAngle:\n{np.round(torque_angle, 2)}",
                        f"Roll:\n{np.round(roll_adjust)}",
                        f"Pitch:\n{np.round(pitch_adjust)}"
                    ]
                else:
                    # Flipper Control
                    flipper_vels = - self.base.flipper_sign * tready_input.flippers * self.FLIPPER_VEL_SCALE

                    if tready_input.torque_toggle:
                        self.base.set_flipper_cmd(p=self.base.flipper_fbk.position, e=np.ones(4) * np.nan)
                    self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)

                    self.torque_labels = None
                
                # Mobile Base Control
                chassis_vels: 'npt.NDArray[np.float64]' = np.array([
                    self.SPEED_MAX_LIN * tready_input.base_motion.x,
                    0,
                    self.SPEED_MAX_ROT * tready_input.base_motion.rz
                ], dtype=np.float64)

                self.base.set_chassis_vel_trajectory(t_now, self.base.chassis_ramp_time, chassis_vels)
        
        self.base.update(t_now, get_feedback=False)

        for handler in self._update_handlers:
            handler(self)

    def transition_to(self, t_now: float, state: TreadyControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print(self.namespace + "TRANSITIONING TO HOMING")
            self.base.home(t_now)
        
        elif state is self.state.ALIGNING:
            print(self.namespace + "TRANSITIONING TO ALIGNING")
            self.base.align_flippers(t_now)

        elif state is self.state.TELEOP:
            print(self.namespace + "TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print(self.namespace + "mobileIO timeout, disabling motion")
            self.base.chassis_traj = None
            self.base.flipper_traj = None
        
        elif state is self.state.EMERGENCY_STOP:
            print(self.namespace + "Emergency Stop Pressed, disabling motion")
            self.base.chassis_traj = None
            self.base.flipper_traj = None

        elif state is self.state.EXIT:
            print(self.namespace + "TRANSITIONING TO EXIT")

        for handler in self._transition_handlers:
            handler(self, state)
        self.state = state
    
    def stop(self):
        self.transition_to(time(), self.state.EXIT)


def config_mobile_io(m: 'MobileIO'):
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

    demo_controller = TreadyControl(base)

    def update_mobile_io(controller: TreadyControl, new_state: TreadyControlState):
        if controller.state == new_state:
            return

        if new_state is controller.state.HOMING:
            controller.base.set_color('magenta')
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(m, msg, color="blue")

        elif new_state is controller.state.TELEOP:
            controller.base.clear_color()
            msg = ('Robot Ready to Control\n'
                   'B1: Reset\n'
                   'B6: Joined Flipper\n'
                   'B8 - Quit')
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

    demo_controller.start_logging()
    last_log_start_time = time()
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

        if t - last_log_start_time > 3600:
            demo_controller.cycle_log()
            last_log_start_time = t

    sys.exit(0)
