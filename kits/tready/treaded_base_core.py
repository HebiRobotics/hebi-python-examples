from dataclasses import dataclass, field
from time import time
from enum import Enum, auto

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as R

import hebi

from .tready_utils import load_gains

import typing

if typing.TYPE_CHECKING:
    from typing import Callable, Self
    from hebi._internal.group import Group


@dataclass
class TreadedBaseConfig:
    hrdf_file: str
    gains_file: str

    wheel_diameter: float  # m
    wheel_base: float  # m

    torso_vel_scale: float = 1.0  # rad/s
    torso_torque_scale: float = 2.5  # Nm
    torque_mode_max: float = 25  # Nm
    torque_ramp_time: float = 8.0  # second ramp up time on torque mode efforts
    torque_angle_offset: float = np.pi / 4

    @property
    def wheel_radius(self):
        return self.wheel_diameter / 2

    @property
    def chassis_rotation_from_wheel(self):
        return self.wheel_base / self.wheel_radius

    @property
    def wheel_to_chassis_vel(self) -> NDArray[np.float64]:
        wr = 2 / self.wheel_base
        return (
            np.array([[1, -1, 1, -1], [0, 0, 0, 0], [wr, wr, wr, wr]])
            * self.wheel_radius
        )

    @property
    def chassis_to_wheel_vel(self) -> NDArray[np.float64]:
        return (
            np.array(
                [
                    [2, 0, self.wheel_base],
                    [-2, 0, self.wheel_base],
                    [2, 0, self.wheel_base],
                    [-2, 0, self.wheel_base],
                ]
            )
            / self.wheel_diameter
        )

    flipper_home_pos: float = np.pi / 2  # rad
    flipper_flat_pos: float = np.pi / 4  # rad
    flipper_rear_pos: NDArray[np.float64] = field(
        default_factory=lambda: np.array(
            [-np.pi / 4, -np.pi / 4, 6.5 * np.pi / 4, 6.5 * np.pi / 4]
        )
    )  # rad

    flipper_max_vel: float = 1.0  # rad/s
    flipper_max_pos: NDArray[np.float64] = field(
        default_factory=lambda: np.array([0.5, np.inf, np.inf, 0.5])
    )
    flipper_min_pos: NDArray[np.float64] = field(
        default_factory=lambda: np.array([-np.inf, -0.5, -0.5, -np.inf])
    )

    def clamp_flipper_position(self, flipper_pos):
        return np.clip(flipper_pos, self.flipper_min_pos, self.flipper_max_pos)

    def clamp_flipper_velocity(self, flipper_vel):
        return np.clip(flipper_vel, -self.flipper_max_vel, self.flipper_max_vel)


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

    def __init__(
        self,
        config: TreadedBaseConfig,
        group: "Group",
        chassis_ramp_time: float,
        flipper_ramp_time: float,
    ):
        self.config = config
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

        self.stow_safe = True
        self.deploy_safe = True
        self.payload_deploy_safe = False
        self.payload_stow_safe = True

        self.internal_drive_safe = False
        self.external_drive_safe = True

        self.ignore_estop = False  # ALWAYS SET FALSE BEFORE RUNNING ON REAL HARDWARE (True for imitation group testing only)

        # final setup before start
        self.set_robot_model(config.hrdf_file)
        load_gains(group, config.gains_file)

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
    def flipper_height(self) -> NDArray[np.float64]:
        x = np.cos(
            self.flipper_sign * self.config.torque_angle_offset
            + self.flipper_fbk.position
        )
        return 1 + np.clip(x, 0, 1) * self.config.chassis_rotation_from_wheel

    @property
    def pose(self) -> NDArray[np.float64]:
        # Use Pose estimate of a single flipper actuator in Tready to get the body Pose estimate
        pos = self.fbk.position
        position = []
        for idx in range(0, 4):
            position.append(pos[idx + 4])
            position.append(pos[idx])

        frames = self.robot_model.get_forward_kinematics_mat("com", position)
        track_rot_mat = frames[10, :3, :3]

        quat = self.fbk.orientation[1]
        q_track = np.array([quat[1], quat[2], quat[3], quat[0]])
        rot_mat_tready = R.from_quat(q_track).as_matrix()
        rot_mat_tready = rot_mat_tready @ track_rot_mat.T

        # convert to euler
        rpy = R.from_matrix(rot_mat_tready).as_euler("xyz", degrees=True)
        return rpy

    @property
    def flipper_max_velocities(self) -> NDArray[np.float64]:
        max_vel = np.full((4), self.config.flipper_max_vel)

        for i in range(4):
            flipper_pos_from_max = (
                self.config.flipper_max_pos[i] - self.flipper_fbk.position[i]
            )
            max_vel[i] = min(max_vel[i], 2 * flipper_pos_from_max)
        return max_vel

    @property
    def flipper_min_velocities(self) -> NDArray[np.float64]:
        min_vel = np.full((4), -self.config.flipper_max_vel)

        for i in range(4):
            flipper_pos_from_min = (
                self.config.flipper_min_pos[i] - self.flipper_fbk.position[i]
            )
            min_vel[i] = max(min_vel[i], 2 * flipper_pos_from_min)
        return min_vel

    @property
    def flipper_wound(self):
        flipper_pos = self.flipper_fbk.position
        flipper_distance = np.abs(
            flipper_pos - self.flipper_sign * self.config.flipper_flat_pos
        )
        flipper_wound = []

        for dist in flipper_distance:
            flipper_wound.append(dist > np.pi / 2)

        return flipper_wound

    def update_feedback(self):
        self.group.get_next_feedback(reuse_fbk=self.fbk)

    def update(self, t_now: float, get_feedback: bool = True):
        if get_feedback:
            self.group.get_next_feedback(reuse_fbk=self.fbk)

        if self.flipper_traj is None and self.chassis_traj is None:
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
                self.wheel_cmd.velocity = (
                    self.config.chassis_to_wheel_vel @ vel
                    + flipper_vels * flipper_height
                )
                for i in range(len(self.wheel_cmd.effort)):
                    if flipper_height[i] > 1 - 1e-12:
                        self.wheel_cmd.effort[i] = np.nan
                    else:
                        self.wheel_cmd.effort[i] = self.flipper_sign[i] * np.tanh(
                            -flipper_height[i]
                            * (1 + self.config.chassis_rotation_from_wheel)
                        )
            else:
                if self.flipper_traj is not None:
                    flipper_height = self.flipper_height
                    flipper_vels = self.flipper_fbk.gyro[:, 2]

                    self.wheel_cmd.position[:] = np.nan
                    self.wheel_cmd.velocity = flipper_vels * flipper_height
                    for i in range(len(self.wheel_cmd.effort)):
                        if flipper_height[i] > 1 - 1e-12:
                            self.wheel_cmd.effort[i] = np.nan
                        else:
                            self.wheel_cmd.effort[i] = self.flipper_sign[i] * np.tanh(
                                -flipper_height[i]
                                * (1 + self.config.chassis_rotation_from_wheel)
                            )
                else:
                    self.wheel_cmd.velocity = 0.0

            if self.flipper_traj is not None:
                # flipper update
                t = min(t_now, self.flipper_traj.end_time)
                [pos, vel, _] = self.flipper_traj.get_state(t)

                self.flipper_cmd.velocity = vel
                self.flipper_cmd.position = pos

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

    def set_flipper_trajectory(
        self, t_now: float, ramp_time: float, p=np.full(4, np.nan), v=np.zeros(4)
    ):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((4, 2), dtype=np.float64)
        velocities = np.empty((4, 2), dtype=np.float64)

        if self.flipper_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            positions[:, 0], velocities[:, 0], _ = self.flipper_traj.get_state(t)
            positions[:, 0] = self.config.clamp_flipper_position(positions[:, 0])
        else:
            positions[:, 0] = self.flipper_fbk.position
            velocities[:, 0] = self.flipper_fbk.velocity

        positions[:, 1] = self.config.clamp_flipper_position(p)
        velocities[:, 1] = np.clip(
            v, self.flipper_min_velocities, self.flipper_max_velocities
        )

        self.flipper_traj = hebi.trajectory.create_trajectory(
            times, positions, velocities
        )

    def set_chassis_vel_trajectory(self, t_now: float, ramp_time: float, v):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((3, 2))
        velocities = np.empty((3, 2))
        accelerations = np.full((3, 2), np.nan)

        if self.chassis_traj is not None:
            t = min(t_now, self.chassis_traj.end_time)
            positions[:, 0], velocities[:, 0], accelerations[:, 0] = (
                self.chassis_traj.get_state(t)
            )
        else:
            positions[:, 0] = 0.0
            velocities[:, 0] = (
                self.config.wheel_to_chassis_vel @ self.wheel_fbk.velocity
            )

        positions[:, 1] = np.nan
        velocities[:, 1] = v
        accelerations[:, 1] = 0.0
        self.chassis_traj = hebi.trajectory.create_trajectory(
            times, positions, velocities, accelerations
        )

    def average_distance(self, a, b):
        return np.mean(np.abs(a - b))

    def move_flippers_to_waypoint(self, t_now: float, avg_speed: float, waypoint):
        flipper_target = self.flipper_sign * waypoint
        ramp_time = (
            self.average_distance(self.flipper_fbk.position, flipper_target) / avg_speed
            + 0.5
        )
        self.set_chassis_vel_trajectory(t_now, 0.25, [0, 0, 0])
        self.set_flipper_trajectory(t_now, ramp_time, p=flipper_target)

    def home(self, t_now: float):
        self.move_flippers_to_waypoint(t_now, 0.2, self.config.flipper_home_pos)

    def rear_up(self, t_now: float):
        self.move_flippers_to_waypoint(t_now, 0.2, self.config.flipper_rear_pos)

    def deploy(self, t_now: float):
        self.set_chassis_vel_trajectory(t_now, 0.25, [0, 0, 0])

    def flatten_flippers(self, t_now: float):
        self.move_flippers_to_waypoint(t_now, 0.2, self.config.flipper_flat_pos)

    def align_flippers(self, t_now: float):
        mean_pos = np.mean(self.flipper_fbk.position * self.flipper_sign)
        aligined_pos = np.full(4, mean_pos)
        self.move_flippers_to_waypoint(t_now, 0.2, aligined_pos)

    def unlock_flippers(self, flippers: list):
        f_positions = []
        f_velocities = []
        f_efforts = []

        t_positions = []
        t_velocities = []
        t_efforts = []

        for flipper in range(4):
            if flipper in flippers:
                f_positions.append(np.nan)
                f_velocities.append(np.nan)
                f_efforts.append(0.0)

                t_positions.append(np.nan)
                t_velocities.append(np.nan)
                t_efforts.append(0.0)
            else:
                if self.flipper_cmd is not None:
                    f_positions.append(self.flipper_cmd.position[flipper])
                    f_velocities.append(self.flipper_cmd.velocity[flipper])
                else:
                    f_positions.append(np.nan)
                    f_velocities.append(np.nan)
                if self.wheel_cmd is not None:
                    t_positions.append(self.wheel_cmd.position[flipper])
                    t_velocities.append(self.wheel_cmd.velocity[flipper])
                else:
                    t_positions.append(np.nan)
                    t_velocities.append(np.nan)

                f_efforts.append(np.nan)
                t_efforts.append(np.nan)

        self.set_flipper_cmd(p=f_positions, v=f_velocities, e=f_efforts)
        self.set_chassis_cmd(p=t_positions, v=t_velocities, e=t_efforts)

    def set_robot_model(self, hrdf_file: str):
        self.robot_model = hebi.robot_model.import_from_hrdf(hrdf_file)

    def set_color(self, color: "hebi.Color | str"):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = hebi.Color(0, 0, 0, 0)
        self.group.send_command(color_cmd)


class TreadyControlState(Enum):
    INITIAL = auto()
    STARTUP = auto()
    HOMING = auto()
    FLATTENING = auto()
    REARING = auto()
    DEPLOYING = auto()
    DEPLOYED = auto()
    STOWING = auto()
    ALIGNING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EMERGENCY_STOP = auto()
    EXIT = auto()

    @property
    def is_error_state(self):
        return self in [
            TreadyControlState.DISCONNECTED,
            TreadyControlState.EMERGENCY_STOP,
        ]


class ChassisVelocity:
    def __init__(self, x: float = 0, rz: float = 0):
        self.x = x
        self.rz = rz

    def __repr__(self) -> str:
        return f"ChassisVelocity(x={self.x}, rz={self.rz})"


@dataclass
class TreadyInputs:
    quit: bool = False
    home: bool = False
    base_motion: ChassisVelocity = field(default_factory=ChassisVelocity)
    flippers: "list[float]" = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    align_flippers: bool = False
    torque_mode: bool = False
    torque_toggle: bool = False
    deploy: bool = False
    deploy_safe: bool = False
    stow: bool = False
    stow_safe: bool = False
    payload_deployed: bool = False
    override_startup: bool = False
    allow_startup: bool = False
    flatten_flippers: bool = False
    drive_safe: bool = False
    rear_up: bool = False

    def __repr__(self) -> str:
        return (
            f"TreadyInputs(home={self.home}, base_motion={self.base_motion}, flippers={self.flippers}, align_flippers={self.align_flippers}, "
            f"torque_mode={self.torque_mode}, torque_toggle={self.torque_toggle}, deploy={self.deploy}, deploy_safe={self.deploy_safe}, "
            f"stow={self.stow} stow_safe={self.stow_safe}, payload_deployed={self.payload_deployed}, override_startup={self.override_startup}, "
            f"allow_startup={self.allow_startup}, flatten_flippers={self.flatten_flippers}, drive_safe={self.drive_safe}, rear_up={self.rear_up})"
        )


class TreadedBaseControl:
    def __init__(self, base: TreadedBase, flipper_vel_scale=1.0, max_base_speed=0.25):
        self.namespace = ""

        self.state = TreadyControlState.INITIAL
        self.prev_state = self.state
        self.base = base

        self.flipper_vel_scale = flipper_vel_scale
        self.max_base_speed = max_base_speed

        self._transition_handlers: "list[Callable[[Self, TreadyControlState], None]]" = []
        self._update_handlers: "list[Callable[[Self, TreadyControlState], None]]" = []

        # Variable for torque mode update handler
        self.torque_labels = None
        self.prev_torque_labels = None
        self.torque_labels_changed = False
        self.torque_start_time = time()
        self.last_cmd_t = time()

        self.allow_payload_startup = False

        self.unlocked_flippers = []
        self.have_wound_flippers = False

    @property
    def max_chassis_rotation_vel(self):
        return self.max_base_speed / (self.base.config.wheel_base / 2)  # rad/s

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

    def update(
        self,
        t_now: float,
        tready_input: "TreadyInputs | None" = None,
    ):
        self.base.update_feedback()

        if self.state is self.state.EXIT:
            return

        if (
            self.base.mstop_pressed
            and self.state is not self.state.EMERGENCY_STOP
            and not self.base.ignore_estop
        ):
            self.transition_to(t_now, self.state.EMERGENCY_STOP)
            return

        # Transition to disconnected if no mobile update recieved
        if tready_input is None:
            if not self.state.is_error_state and (t_now - self.last_cmd_t) > 1.0:
                print(self.namespace + "mobileIO timeout, base disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return

        # Reset the timeout
        self.last_cmd_t = t_now

        # Update deploy/stow/drive safety
        self.base.deploy_safe = tready_input.deploy_safe
        self.base.stow_safe = tready_input.stow_safe
        self.base.external_drive_safe = tready_input.drive_safe

        if self.state is self.state.EMERGENCY_STOP:
            if not self.base.mstop_pressed:
                print(self.namespace + "Emergency Stop Released")
                self.transition_to(t_now, self.prev_state)

        # Transition to previous state if mobileIO is reconnected
        elif self.state is self.state.DISCONNECTED:
            self.last_cmd_t = t_now
            print(self.namespace + "Controller reconnected, demo continued.")
            self.transition_to(t_now, self.prev_state)

        # On first loop move to startup
        elif self.state is self.state.INITIAL:
            self.transition_to(t_now, self.state.STARTUP)

        # If homing/aligning/flattening is complete, transition to teleop
        elif (
            self.state is self.state.HOMING
            or self.state is self.state.ALIGNING
            or self.state is self.state.FLATTENING
            or self.state is self.state.REARING
        ):
            if not self.base.has_active_flipper_trajectory:
                self.transition_to(t_now, self.state.TELEOP)

        # If deploying is complete, transition to deployed
        elif self.state is self.state.DEPLOYING:
            if not self.base.has_active_trajectory:
                self.transition_to(t_now, self.state.DEPLOYED)

        # Transition to teleop if safe
        elif self.state is self.state.STOWING:
            if self.base.stow_safe:
                self.transition_to(t_now, self.state.TELEOP)

        # Transition to teleop when startup complete
        elif self.state is self.state.STARTUP:
            if True in self.base.flipper_wound and not self.have_wound_flippers:
                self.have_wound_flippers = True
                self.unlocked_flippers = []
                for i in range(4):
                    if self.base.flipper_wound[i]:
                        self.unlocked_flippers.append(i)

            if (
                not self.have_wound_flippers
                or tready_input.override_startup
                or self.allow_payload_startup
            ):
                self.allow_payload_startup = True
                if tready_input.allow_startup:
                    self.transition_to(t_now, self.state.TELEOP)

            else:
                self.base.unlock_flippers(self.unlocked_flippers)

        # Deployed mode
        elif self.state is self.state.DEPLOYED:
            # Check for stow
            if tready_input.stow:
                self.transition_to(t_now, self.state.STOWING)

        # Teleop mode
        elif self.state is self.state.TELEOP:
            # IDK why but sometimes this happens
            if not tready_input.allow_startup:
                print("How did this happen")
                self.transition_to(t_now, self.state.STARTUP)
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
            # Check for flipper flatten
            elif tready_input.flatten_flippers:
                if tready_input.torque_mode:
                    print(self.namespace + "Cannot flatten flippers in torque mode")
                    return
                self.transition_to(t_now, self.state.FLATTENING)
            # Check for rear up
            elif tready_input.rear_up:
                if tready_input.torque_mode:
                    print(self.namespace + "Cannot rear up in torque mode")
                    return
                self.transition_to(t_now, self.state.REARING)
            # Check for deployment
            elif tready_input.deploy:
                if tready_input.torque_mode:
                    print(self.namespace + "Cannot deploy in torque mode")
                    return
                self.transition_to(t_now, self.state.DEPLOYING)
            else:
                if tready_input.torque_mode:
                    if tready_input.torque_toggle:
                        self.torque_start_time = t_now

                    torque_max = self.base.config.torque_mode_max * (
                        (tready_input.flippers[0] + 1) / 2
                    )
                    torque_max_label = torque_max
                    torque_max *= min(
                        (t_now - self.torque_start_time)
                        / self.base.config.torque_ramp_time,
                        1,
                    )
                    torque_angle = (
                        1 + tready_input.flippers[1]
                    ) * self.base.config.torque_angle_offset
                    roll_angle, pitch_angle, _ = self.base.pose

                    roll_adjust = (tready_input.flippers[2] + 1) / 2
                    pitch_adjust = (tready_input.flippers[3] + 1) / 2

                    roll_signs = [0, -1, 0, 1] if roll_angle > 0 else [-1, 0, 1, 0]
                    roll_torque = (
                        np.array(roll_signs)
                        * roll_angle
                        * self.base.config.torso_torque_scale
                        * roll_adjust
                    )

                    pitch_signs = [1, -1, 0, 0] if pitch_angle > 0 else [0, 0, 1, -1]
                    pitch_torque = (
                        np.array(pitch_signs)
                        * pitch_angle
                        * self.base.config.torso_torque_scale
                        * pitch_adjust
                    )

                    level_torque = roll_torque + pitch_torque
                    flipper_efforts = (
                        -np.tanh(
                            self.base.flipper_fbk.position
                            - self.base.flipper_sign * torque_angle
                        )
                        * torque_max
                        + level_torque
                    )

                    self.base.flipper_traj = None
                    self.base.set_flipper_cmd(
                        p=np.full(4, np.nan), v=np.full(4, np.nan), e=flipper_efforts
                    )

                    self.prev_torque_labels = self.torque_labels
                    self.torque_labels = [
                        f"Max\nEff:\n{np.round(torque_max_label, 2)}",
                        f"Torque\nAngle:\n{np.round(torque_angle, 2)}",
                        f"Roll:\n{np.round(roll_adjust)}",
                        f"Pitch:\n{np.round(pitch_adjust)}",
                    ]
                    self.torque_labels_changed = (
                        self.prev_torque_labels != self.torque_labels
                    )

                else:
                    # Flipper Control
                    flipper_vels = (
                        -self.base.flipper_sign
                        * tready_input.flippers
                        * self.flipper_vel_scale
                    )

                    if tready_input.torque_toggle:
                        self.base.set_flipper_cmd(
                            p=self.base.flipper_fbk.position, e=np.full(4, np.nan)
                        )
                    self.base.set_flipper_trajectory(
                        t_now, self.base.flipper_ramp_time, v=flipper_vels
                    )
                    self.torque_labels = None

        if self.base.internal_drive_safe and self.base.external_drive_safe:
            # Mobile Base Control
            chassis_vels: NDArray[np.float64] = np.array(
                [
                    self.max_base_speed * tready_input.base_motion.x,
                    0,
                    self.max_chassis_rotation_vel * tready_input.base_motion.rz,
                ],
                dtype=np.float64,
            )

            self.base.set_chassis_vel_trajectory(
                t_now, self.base.chassis_ramp_time, chassis_vels
            )
        else:
            self.base.chassis_traj = None

        self.base.update(t_now, get_feedback=False)

        for handler in self._update_handlers:
            handler(self, self.state)

    def transition_to(self, t_now: float, state: TreadyControlState):
        # self transitions are noop
        if state == self.state:
            return
        self.prev_state = self.state

        if state is self.state.HOMING:
            print(self.namespace + "BASE TRANSITIONING TO HOMING")
            self.base.home(t_now)
            self.base.internal_drive_safe = False

        elif state is self.state.FLATTENING:
            print(self.namespace + "BASE TRANSITIONING TO FLATTENING")
            self.base.flatten_flippers(t_now)
            self.base.internal_drive_safe = False

        elif state is self.state.REARING:
            print(self.namespace + "BASE TRANSITIONING TO REARING")
            self.base.rear_up(t_now)
            self.base.internal_drive_safe = False

        elif state is self.state.STARTUP:
            print(self.namespace + "BASE TRANSITIONING TO STARTUP")
            self.base.internal_drive_safe = False

        elif state is self.state.ALIGNING:
            print(self.namespace + "BASE TRANSITIONING TO ALIGNING")
            self.base.align_flippers(t_now)
            self.base.internal_drive_safe = False

        elif state is self.state.DEPLOYING:
            print(self.namespace + "BASE TRANSITIONING TO DEPLOYING")
            self.base.deploy(t_now)
            self.base.internal_drive_safe = False

        elif state is self.state.STOWING:
            print(self.namespace + "BASE TRANSITIONING TO STOWING")
            self.base.internal_drive_safe = False

        elif state is self.state.DEPLOYED:
            print(self.namespace + "BASE TRANSITIONING TO DEPLOYED")
            self.base.payload_deploy_safe = True
            self.base.payload_stow_safe = True
            self.base.internal_drive_safe = True

        elif state is self.state.TELEOP:
            print(self.namespace + "BASE TRANSITIONING TO TELEOP")
            self.base.payload_deploy_safe = False
            self.base.payload_stow_safe = False
            self.base.internal_drive_safe = True

        elif state is self.state.DISCONNECTED:
            print(self.namespace + "BASE DISCONNECTED")
            self.base.chassis_traj = None
            self.base.flipper_traj = None
            self.base.internal_drive_safe = False

        elif state is self.state.EMERGENCY_STOP:
            print(self.namespace + "BASE EMERGENCY_STOP")
            self.base.chassis_traj = None
            self.base.flipper_traj = None
            self.base.internal_drive_safe = False

        elif state is self.state.EXIT:
            print(self.namespace + "BASE EXIT")

        for handler in self._transition_handlers:
            handler(self, state)
        self.state = state

    def stop(self):
        self.transition_to(time(), self.state.EXIT)
