from __future__ import annotations

from enum import Enum, auto
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

import hebi


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

    flipper_home_pos: float = np.pi / 2  # rad
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
    COMPLIANT_FLIPPERS = auto()
    DISCONNECTED = auto()
    EMERGENCY_STOP = auto()
    EXIT = auto()

    @property
    def is_error_state(self):
        return self in [
            TreadyControlState.DISCONNECTED,
            TreadyControlState.EMERGENCY_STOP,
        ]

