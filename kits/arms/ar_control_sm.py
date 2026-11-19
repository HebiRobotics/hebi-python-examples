"""Control an arm by moving a phone running Mobile IO (AR tracking).

States: Startup -> Idle <-> Teleop, and Homing on the home button. The arm
holds where it is on startup and only moves on user input. Unlock (lock
button) in Idle to start tracking the phone; lock again to stop. Input loss
holds position and resumes in Idle, so tracking always restarts from a fresh
phone reference.

    python -m kits.arms.ar_control_sm A-2580-06G.cfg.yaml
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from time import sleep
from typing import TYPE_CHECKING

import hebi
import numpy as np
from hebi.util import create_mobile_io
from scipy.spatial.transform import Rotation as R

from kits.common.config import add_config_argument, config_arg_parser, load_config, required_user_data
from kits.common.state_machine import ControllerState

from .arm_controller import CONFIG_DIR, ArmController, ArmExitState, ArmFaultState, find_gripper, user_data_value

if TYPE_CHECKING:
    from collections.abc import Sequence

    from hebi._internal.mobile_io import MobileIO
    from hebi.arm import Arm, Gripper
    from numpy.typing import NDArray


@dataclass(kw_only=True)
class ArmMobileIOInputs:
    phone_pos: NDArray[np.float32]
    phone_rot: NDArray[np.float64]
    ar_scaling: float = 1.0
    lock_toggle: bool = False
    locked: bool = True
    gripper_closed: bool = False
    home: bool = False


class _ResumeToIdle:
    controller: ArmMobileIOController

    def resume_state(self):
        return ARIdleState(self.controller)


@dataclass(eq=False)
class ARStartupState(ControllerState["ArmMobileIOController"]):
    # Never move on startup: the arm may be in an unusual pose after a crash.
    def update(self, t_now, **kwargs):
        return ARIdleState(self.controller)


@dataclass(eq=False)
class ARHomingState(_ResumeToIdle, ControllerState["ArmMobileIOController"]):
    led_color = "magenta"

    def on_enter(self, from_state, **kwargs):
        self.controller.move_to(self.controller.arm_home, self.controller.homing_time)

    def update(self, t_now, **kwargs):
        if self.controller.arm.at_goal:
            return ARIdleState(self.controller)
        return None

    def receive_inputs(self, t_now, inputs: ArmMobileIOInputs, **kwargs):
        self.controller.set_gripper(inputs.gripper_closed)


@dataclass(eq=False)
class ARIdleState(_ResumeToIdle, ControllerState["ArmMobileIOController"]):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state, **kwargs):
        self.controller.hold_position()

    def receive_inputs(self, t_now, inputs: ArmMobileIOInputs, **kwargs):
        self.controller.set_gripper(inputs.gripper_closed)
        if self.requested is not None:
            return

        if inputs.home:
            self.requested = ARHomingState(self.controller)
        elif inputs.lock_toggle and not inputs.locked:
            teleop = ARTeleopState(self.controller)
            teleop.set_reference(inputs, self.controller.current_position())
            self.requested = teleop

    def update(self, t_now, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class ARTeleopState(_ResumeToIdle, ControllerState["ArmMobileIOController"]):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    # Phone pose when tracking started, and the arm pose it maps to.
    phone_xyz_home: NDArray[np.float64] = field(init=False)
    phone_rot_home: NDArray[np.float64] = field(init=False)
    locked_xyz: NDArray[np.float64] = field(init=False)
    locked_rot: NDArray[np.float64] = field(init=False)
    locked_seed: NDArray[np.float64] = field(init=False)

    def set_reference(self, inputs: ArmMobileIOInputs, arm_position: NDArray[np.float64]):
        self.phone_xyz_home = np.array(inputs.phone_pos, dtype=np.float64)
        self.phone_rot_home = inputs.phone_rot
        self.locked_xyz = np.empty(3)
        self.locked_rot = np.empty((3, 3))
        self.controller.arm.FK(arm_position, xyz_out=self.locked_xyz, orientation_out=self.locked_rot)
        self.locked_seed = arm_position.copy()

    def receive_inputs(self, t_now, inputs: ArmMobileIOInputs, **kwargs):
        self.controller.set_gripper(inputs.gripper_closed)
        if self.requested is not None:
            return

        if inputs.home:
            self.requested = ARHomingState(self.controller)
        elif inputs.lock_toggle and inputs.locked:
            self.requested = ARIdleState(self.controller)
        elif (joint_target := self.compute_target(inputs)) is not None:
            self.controller.move_to(joint_target, self.controller.traj_duration)

    def update(self, t_now, **kwargs):
        requested, self.requested = self.requested, None
        return requested

    def compute_target(self, inputs: ArmMobileIOInputs) -> NDArray[np.float64] | None:
        if np.any(np.isnan(inputs.phone_pos)) or np.any(np.isnan(inputs.phone_rot)):
            print(f'{self.controller.prefix}NaN in AR pose, skipping sample')
            return None

        phone_offset = inputs.phone_pos - self.phone_xyz_home
        rot_mat = self.phone_rot_home
        xyz_target = self.locked_xyz + inputs.ar_scaling * self.controller.xyz_scale * (rot_mat.T @ phone_offset)
        rot_target = rot_mat.T @ inputs.phone_rot @ self.locked_rot

        # With scaling at 0, keep re-zeroing the phone reference so the arm doesn't
        # jump to an accumulated offset when scaling is turned back up.
        if inputs.ar_scaling == 0.0:
            self.phone_xyz_home = np.array(inputs.phone_pos, dtype=np.float64)

        try:
            return self.controller.arm.ik_target_xyz_so3(self.locked_seed, xyz_target, rot_target)
        except Exception as e:  # noqa: BLE001 - skip one bad IK sample rather than end the demo
            print(f'{self.controller.prefix}IK failed ({e}) for xyz={xyz_target}, skipping sample')
            return None


class ArmMobileIOController(ArmController[ArmMobileIOInputs]):
    """Arm teleoperated by a phone's AR pose. Homes to a joint-space ``home_position``."""

    def __init__(
        self,
        arm: Arm,
        home_position: Sequence[float] | NDArray[np.float64],
        gripper: Gripper | None = None,
        *,
        homing_time: float = 5.0,
        traj_duration: float = 1.0,
        xyz_scale: NDArray[np.float64] | None = None,
        name: str = "Arm",
        input_timeout: float | None = 1.0,
    ):
        super().__init__(arm, ARStartupState, ArmFaultState, ArmExitState, gripper=gripper, name=name, input_timeout=input_timeout)
        self.arm_home = np.asarray(home_position, dtype=np.float64)
        self.homing_time = homing_time
        self.traj_duration = traj_duration
        self.xyz_scale = np.ones(3) if xyz_scale is None else np.asarray(xyz_scale, dtype=np.float64)

    @classmethod
    def from_config(cls, arm: Arm, cfg: hebi.config.HebiConfig, gripper: Gripper | None = None, **kwargs):
        """Read ``home_position`` (required), homing time, trajectory duration and ``xyz_scale`` from ``cfg.user_data``.

        Accepts both key spellings used in the kit configs
        (``homing_duration``/``homing_time``, ``delay_time``/``trajectory_duration``).
        Keyword arguments override the config.
        """
        params = {
            "home_position": required_user_data(cfg, "home_position"),
            "homing_time": float(user_data_value(cfg, "homing_time", "homing_duration", default=5.0)),
            "traj_duration": float(user_data_value(cfg, "trajectory_duration", "delay_time", default=1.0)),
            "xyz_scale": user_data_value(cfg, "xyz_scale", default=None),
        }
        params.update(kwargs)
        return cls(arm, gripper=gripper, **params)


def phone_rotation(m: MobileIO) -> NDArray[np.float64]:
    try:
        wxyz = m.orientation  # reorder by hand: from_quat(scalar_first=...) needs scipy >= 1.14
        return R.from_quat([*wxyz[1:], wxyz[0]]).as_matrix()
    except ValueError as e:
        print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
        # Invalid pose: the arm holds position rather than tracking a made-up orientation
        return np.full((3, 3), np.nan)


def setup_mobile_io(m: MobileIO):
    m.set_button_label(1, '⟲')
    m.set_button_label(5, 'lock')
    m.set_button_mode(5, 1)
    m.set_button_label(7, 'grip')
    m.set_button_mode(7, 1)
    m.set_button_label(8, '❌')


def parse_mobile_feedback(m: MobileIO, timeout_ms: float = 100.0):
    """Returns ``(quit, inputs)``; ``inputs`` is ``None`` if no new sample arrived."""
    if not m.update(timeout_ms):
        return False, None

    if m.get_button_state(8):
        return True, None

    # Button 5 is a toggle: on = unlocked (tracking), off = locked.
    arm_input = ArmMobileIOInputs(
        home=bool(m.get_button_state(1)),
        phone_pos=np.copy(m.position),
        phone_rot=phone_rotation(m),
        lock_toggle=m.get_button_diff(5) != 0,
        locked=m.get_button_state(5) == 0,
        gripper_closed=bool(m.get_button_state(7)),
    )
    return False, arm_input


if __name__ == "__main__":
    parser = config_arg_parser("Control an arm by moving a phone (Mobile IO AR)", CONFIG_DIR)
    add_config_argument(parser, "config", "Arm config, e.g. A-2580-06G.cfg.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config, CONFIG_DIR)
    lookup = hebi.Lookup()
    sleep(2)

    # Find all hardware before anything starts moving
    arm = hebi.arm.create_from_config(cfg, lookup)
    arm_control = ArmMobileIOController.from_config(arm, cfg, find_gripper(lookup, cfg))

    print('Looking for mobileIO device...')
    m = create_mobile_io(lookup, cfg.families[0])
    while m is None:
        try:
            print('Waiting for mobileIO device to come online...')
            sleep(1)
            m = create_mobile_io(lookup, cfg.families[0])
        except KeyboardInterrupt:
            sys.exit()

    print("mobileIO device found.")
    m.resetUI()
    m.update()
    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    arm_control.start()
    try:
        while arm_control.running:
            quit, arm_input = parse_mobile_feedback(m)
            if quit:
                break
            if arm_input is not None:
                arm_control.receive_inputs(arm_input)
    except KeyboardInterrupt:
        pass
    finally:
        arm_control.stop()
