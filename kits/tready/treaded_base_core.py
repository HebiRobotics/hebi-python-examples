from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from scipy.spatial.transform import Rotation as R

import time

import hebi
from hebi.arm import GroupManager

from kits.common.state_machine import Controller, ControllerState, wait_for_group

from .tready_utils import load_gains

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.group import Group
    from hebi._internal.lookup import Lookup


@dataclass
class TreadedBaseConfig:
    hrdf_file: str
    gains_file: str

    wheel_diameter: float  # m
    wheel_base: float  # m
    flipper_home_pos: float  # rad

    torso_vel_scale: float = 1.0  # rad/s
    torso_torque_scale: float = 2.5  # Nm
    torque_mode_max: float = 25  # Nm
    torque_ramp_time: float = 8.0  # second ramp up time on torque mode efforts
    torque_angle_offset: float = np.pi / 4

    @classmethod
    def from_hebi_config(cls, cfg: hebi.config.HebiConfig):
        assert cfg.hrdf is not None
        assert cfg.gains is not None
        assert cfg.user_data is not None

        return cls(
            hrdf_file=cfg.hrdf,
            gains_file=cfg.gains["default"],
            wheel_diameter=cfg.user_data["wheel_diameter"],
            wheel_base=cfg.user_data["wheel_base"],
            flipper_home_pos=cfg.user_data["flipper_home_position"],
            torso_torque_scale=cfg.user_data["torso_torque_scale"],
            torque_mode_max=cfg.user_data["torque_mode_max"],
        )

    @property
    def wheel_radius(self):
        return self.wheel_diameter / 2

    @property
    def chassis_rotation_from_wheel(self):
        return self.wheel_base / self.wheel_radius

    @property
    def wheel_to_chassis_vel(self) -> NDArray[np.float64]:
        wr = 2 / self.wheel_base
        return np.array([[1, -1, 1, -1], [0, 0, 0, 0], [wr, wr, wr, wr]]) * self.wheel_radius

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

    flipper_flat_pos: float = np.pi / 4  # rad
    flipper_rear_pos: NDArray[np.float64] = field(
        default_factory=lambda: np.array([-np.pi / 4, -np.pi / 4, 6.5 * np.pi / 4, 6.5 * np.pi / 4])
    )  # rad

    flipper_max_vel: float = 1.0  # rad/s
    flipper_max_pos: NDArray[np.float64] = field(default_factory=lambda: np.array([0.5, np.inf, np.inf, 0.5]))
    flipper_min_pos: NDArray[np.float64] = field(default_factory=lambda: np.array([-np.inf, -0.5, -0.5, -np.inf]))

    def clamp_flipper_position(self, flipper_pos):
        return np.clip(flipper_pos, self.flipper_min_pos, self.flipper_max_pos)

    def clamp_flipper_velocity(self, flipper_vel):
        return np.clip(flipper_vel, -self.flipper_max_vel, self.flipper_max_vel)


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
    flippers: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    align_flippers: bool = False
    torque_mode: bool = False
    torque_toggle: bool = False
    deploy: bool = False
    deploy_safe: bool = True
    stow: bool = False
    stow_safe: bool = True
    payload_deployed: bool = False
    override_startup: bool = False
    allow_startup: bool = True
    flatten_flippers: bool = False
    drive_safe: bool = True
    rear_up: bool = False

    def __repr__(self) -> str:
        return (
            f"TreadyInputs(home={self.home}, base_motion={self.base_motion}, flippers={self.flippers}, align_flippers={self.align_flippers}, "
            f"torque_mode={self.torque_mode}, torque_toggle={self.torque_toggle}, deploy={self.deploy}, deploy_safe={self.deploy_safe}, "
            f"stow={self.stow} stow_safe={self.stow_safe}, payload_deployed={self.payload_deployed}, override_startup={self.override_startup}, "
            f"allow_startup={self.allow_startup}, flatten_flippers={self.flatten_flippers}, drive_safe={self.drive_safe}, rear_up={self.rear_up})"
        )


@dataclass(eq=False)
class TreadyEStopState(ControllerState["TreadedBaseController"]):
    # Drop all motion targets: a flipper target held through the M-stop would
    # be driven to on release. On recovery, teleop restarts velocity control
    # from measured positions.
    def on_enter(self, from_state, **kwargs):
        self.controller.base.set_color("yellow")
        self.controller.base.clear_motion()


@dataclass(eq=False)
class TreadyExitState(ControllerState["TreadedBaseController"]):
    # Runs before Controller.stop() joins the control thread, so it must not
    # call base.stop() itself.
    def on_enter(self, from_state, **kwargs):
        self.controller.base.set_color("red")
        self.controller.base.hold_motion()


class TreadedBaseController(Controller["TreadedBase", TreadyInputs]):
    """State machine for a treaded base. States live in ``treaded_base.py``.

    Call ``start()`` after registering handlers; nothing runs before that.
    """

    def __init__(
        self,
        base: TreadedBase,
        starting_state: type[ControllerState],
        fault_state: type[ControllerState],
        exit_state: type[ControllerState],
        flipper_vel_scale=1.0,
        max_base_speed=0.25,
        *,
        name: str = "Base",
        input_timeout: float | None = 1.0,
    ):
        super().__init__(base, starting_state, fault_state, exit_state, input_timeout=input_timeout, name=name)

        self.flipper_vel_scale = flipper_vel_scale
        self.max_base_speed = max_base_speed

    @property
    def base(self) -> TreadedBase:
        return self.manager

    @property
    def max_chassis_rotation_vel(self):
        return self.max_base_speed / (self.base.config.wheel_base / 2)  # rad/s

    def drive(self, tready_input: TreadyInputs):
        """Command chassis velocity from the joystick, or ramp to a stop if driving is unsafe."""
        if tready_input.drive_safe:
            chassis_vels: NDArray[np.float64] = np.array(
                [
                    self.max_base_speed * tready_input.base_motion.x,
                    0,
                    self.max_chassis_rotation_vel * tready_input.base_motion.rz,
                ],
                dtype=np.float64,
            )
            duration = self.base.chassis_ramp_time
        else:
            chassis_vels = np.zeros(3)
            duration = 0.2

        self.base.set_chassis_vel_trajectory(duration, chassis_vels)

    def on_disconnect(self):
        self.base.set_color("blue")
        self.base.hold_motion()


class TreadedBase(GroupManager):
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

    def __init__(self, config: TreadedBaseConfig, group: Group, chassis_ramp_time: float, flipper_ramp_time: float):
        super().__init__(time.time, group)

        self.config = config

        self.wheel_fbk = self.last_feedback.create_view([0, 1, 2, 3])
        self.flipper_fbk = self.last_feedback.create_view([4, 5, 6, 7])

        self.wheel_cmd = self.pending_command.create_view([0, 1, 2, 3])
        self.flipper_cmd = self.pending_command.create_view([4, 5, 6, 7])

        self.flipper_sign = np.array([-1, 1, 1, -1])

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.flipper_cmd.position = self.flipper_fbk.position
        self.wheel_cmd.position = np.nan

        self._aligned_flipper_mode = False

        self.chassis_traj = None
        self.flipper_traj = None
        self.user_flipper_torque = np.full(4, np.nan)

        self.robot_model = None

        # final setup before start
        self.set_robot_model(self.config.hrdf_file)
        load_gains(group, self.config.gains_file)

    @classmethod
    def from_config(cls, cfg: hebi.config.HebiConfig, lookup: Lookup | None = None):
        if lookup is None:
            lookup = hebi.Lookup()
        base_group = wait_for_group(lookup, cfg.families, cfg.names)

        assert cfg.user_data is not None

        return cls(
            TreadedBaseConfig.from_hebi_config(cfg),
            base_group,
            float(cfg.user_data["chassis_ramp_time"]),
            float(cfg.user_data["flipper_ramp_time"]),
        )

    @property
    def has_active_base_trajectory(self):
        if self.chassis_traj is not None and self._last_time < self.chassis_traj.end_time:
            return True
        return False

    @property
    def has_active_flipper_trajectory(self):
        if self.flipper_traj is not None and self._last_time < self.flipper_traj.end_time:
            return True
        return False

    @property
    def has_active_trajectory(self):
        return self.has_active_base_trajectory or self.has_active_flipper_trajectory

    @property
    def flipper_height(self) -> NDArray[np.float64]:
        x = np.cos(self.flipper_sign * self.config.torque_angle_offset + self.flipper_fbk.position)
        return 1 + np.clip(x, 0, 1) * self.config.chassis_rotation_from_wheel

    @property
    def pose(self) -> NDArray[np.float64]:
        # Use Pose estimate of a single flipper actuator in Tready to get the body Pose estimate
        pos = self.last_feedback.position
        position = []
        for idx in range(0, 4):
            position.append(pos[idx + 4])
            position.append(pos[idx])

        frames = self.robot_model.get_forward_kinematics_mat("com", position)
        track_rot_mat = frames[10, :3, :3]

        quat = self.last_feedback.orientation[1]
        if np.any(np.isnan(quat)):  # no IMU orientation yet: report level
            return np.zeros(3)
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
            flipper_pos_from_max = self.config.flipper_max_pos[i] - self.flipper_fbk.position[i]
            max_vel[i] = min(max_vel[i], 2 * flipper_pos_from_max)
        return max_vel

    @property
    def flipper_min_velocities(self) -> NDArray[np.float64]:
        min_vel = np.full((4), -self.config.flipper_max_vel)

        for i in range(4):
            flipper_pos_from_min = self.config.flipper_min_pos[i] - self.flipper_fbk.position[i]
            min_vel[i] = max(min_vel[i], 2 * flipper_pos_from_min)
        return min_vel

    @property
    def flipper_wound(self):
        flipper_pos = self.flipper_fbk.position
        flipper_distance = np.abs(flipper_pos - self.flipper_sign * self.config.flipper_flat_pos)
        flipper_wound = []

        for dist in flipper_distance:
            flipper_wound.append(dist > np.pi / 2)

        return flipper_wound

    def _inner_update(self, t: float) -> bool:
        res = super()._inner_update(t)

        if self.flipper_traj is None and self.chassis_traj is None:
            self.pending_command.velocity = 0.0
        else:
            if self.flipper_traj is not None:
                # flipper update
                [pos, vel, _] = self.flipper_traj.get_state(t)

                self.flipper_cmd.position = pos
                self.flipper_cmd.velocity = vel
                self.flipper_cmd.effort = np.nan
            else:
                self.flipper_cmd.position = np.nan
                self.flipper_cmd.velocity = np.nan
                self.flipper_cmd.effort = self.user_flipper_torque

            # Moving average setup below, in an attempt to make Tready less wobbly on tiptoes
            flipper_vels = -1.0 * self.flipper_fbk.velocity_command
            if np.any(np.isnan(flipper_vels)):
                flipper_vels = self.flipper_fbk.gyro[:, 2]

            self.wheel_cmd.position = np.nan
            self.wheel_cmd.velocity = flipper_vels * self.flipper_height
            if self.chassis_traj is not None:
                # chassis update
                [_, vel, _] = self.chassis_traj.get_state(t)
                self.wheel_cmd.velocity += self.config.chassis_to_wheel_vel @ vel

            for i in range(len(self.wheel_cmd.effort)):
                if self.flipper_height[i] > 1 - 1e-12:
                    self.wheel_cmd.effort[i] = np.nan
                else:
                    self.wheel_cmd.effort[i] = self.flipper_sign[i] * np.tanh(
                        -self.flipper_height[i] * (1 + self.config.chassis_rotation_from_wheel)
                    )

        return res

    def clear_motion(self):
        """Drop flipper and chassis trajectories and flipper torques; the next update commands zero velocity."""
        self.flipper_traj = None
        self.chassis_traj = None
        self.user_flipper_torque[:] = np.nan

    def hold_motion(self, ramp_time: float = 0.2):
        """Ramp the chassis to a stop and hold the flippers where they are."""
        flipper_pos = self.flipper_fbk.position_command
        if np.any(np.isnan(flipper_pos)):
            flipper_pos = self.flipper_fbk.position
        self.set_flipper_trajectory(ramp_time, flipper_pos)
        self.set_chassis_vel_trajectory(ramp_time, np.zeros(3))

    def set_flipper_torque(self, e: NDArray[np.float64]):
        self.flipper_traj = None
        self.user_flipper_torque = e

    def set_flipper_trajectory(self, ramp_time: float, p=np.full(4, np.nan), v=np.zeros(4)):

        t_now = time.time()
        if self.flipper_traj is not None:
            p_now, v_now, _ = self.flipper_traj.get_state(t_now)
            p_now = self.config.clamp_flipper_position(p_now)
        else:
            p_now = self.flipper_fbk.position
            v_now = self.flipper_fbk.velocity

        g = hebi.Goal(4)
        g.add_waypoint(
            ramp_time, self.config.clamp_flipper_position(p), np.clip(v, self.flipper_min_velocities, self.flipper_max_velocities)
        )
        self.user_flipper_torque[:] = np.nan
        self.flipper_traj, _ = g.build_trajectory_from(t_now, p_now, v_now.astype(np.float64))

    def set_chassis_vel_trajectory(self, ramp_time: float, v):
        t_now = time.time()

        if self.chassis_traj is not None:
            p_now, v_now, a_now = self.chassis_traj.get_state(t_now)
        else:
            p_now = np.zeros(3)
            v_now = self.config.wheel_to_chassis_vel @ self.wheel_fbk.velocity
            a_now = np.zeros(3)

        g = hebi.Goal(3)
        g.add_waypoint(ramp_time, np.full(3, np.nan), v)

        self.chassis_traj, _ = g.build_trajectory_from(t_now, p_now, v_now, a_now)

    def average_distance(self, a, b):
        return np.mean(np.abs(a - b))

    def move_flippers_to_waypoint(self, avg_speed: float, waypoint):
        flipper_target = self.flipper_sign * waypoint
        self.set_chassis_vel_trajectory(0.25, [0, 0, 0])
        self.set_flipper_trajectory(1.0, p=flipper_target)
        self.flipper_traj.rescale_with_limits(np.full(4, avg_speed), None)

    def home(self):
        self.move_flippers_to_waypoint(0.75, self.config.flipper_home_pos)

    def rear_up(self):
        self.move_flippers_to_waypoint(0.75, self.config.flipper_rear_pos)

    def deploy(self):
        self.set_chassis_vel_trajectory(0.25, [0, 0, 0])

    def flatten_flippers(self):
        self.move_flippers_to_waypoint(0.75, self.config.flipper_flat_pos)

    def align_flippers(self):
        mean_pos = np.mean(self.flipper_fbk.position * self.flipper_sign)
        aligned_pos = np.full(4, mean_pos)
        self.move_flippers_to_waypoint(0.75, aligned_pos)

    def unlock_flippers(self, flippers: list):
        f_positions = self.flipper_cmd.position.copy()
        f_velocities = self.flipper_cmd.velocity.copy()
        f_efforts = self.flipper_cmd.effort.copy()

        t_positions = self.wheel_cmd.position.copy()
        t_velocities = self.wheel_cmd.velocity.copy()
        t_efforts = self.wheel_cmd.effort.copy()

        f_positions[flippers] = np.nan
        f_velocities[flippers] = np.nan
        f_efforts[flippers] = 0.0

        t_positions[flippers] = np.nan
        t_velocities[flippers] = np.nan
        t_efforts[flippers] = 0.0

        self.flipper_cmd.position = f_positions
        self.flipper_cmd.velocity = f_velocities
        self.flipper_cmd.effort = f_efforts

        self.wheel_cmd.position = t_positions
        self.wheel_cmd.velocity = t_velocities
        self.wheel_cmd.effort = t_efforts

    def set_robot_model(self, hrdf_file: str):
        self.robot_model = hebi.robot_model.import_from_hrdf(hrdf_file)

    def set_color(self, color: hebi.Color | str):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = hebi.Color(0, 0, 0, 0)
        self.group.send_command(color_cmd)
