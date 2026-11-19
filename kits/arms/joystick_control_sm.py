"""Jog an arm's end effector with Mobile IO joysticks.

States: Startup -> Teleop, and Homing on the home button. The arm holds where
it is on startup and only moves on user input. Input loss holds position and
resumes in Teleop (re-seeded from the current pose).

    python -m kits.arms.joystick_control_sm A-2580-06G.cfg.yaml [--no-gripper]
"""

from __future__ import annotations

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


@dataclass
class ArmJoystickInputs:
    home: bool
    delta_xyz: NDArray[np.float64]
    delta_rot_xyz: NDArray[np.float64]
    gripper_closed: bool = False

    def __post_init__(self):
        self.delta_xyz = np.asarray(self.delta_xyz, dtype=np.float64)
        self.delta_rot_xyz = np.asarray(self.delta_rot_xyz, dtype=np.float64)

    @classmethod
    def idle(cls) -> ArmJoystickInputs:
        """No motion; used while another part of the robot is starting up."""
        return cls(home=False, delta_xyz=np.zeros(3), delta_rot_xyz=np.zeros(3))


class _ResumeToTeleop:
    controller: ArmJoystickController

    def resume_state(self):
        return JoystickTeleopState(self.controller)


@dataclass(eq=False)
class JoystickStartupState(ControllerState["ArmJoystickController"]):
    # Never move on startup: the arm may be in an unusual pose after a crash.
    def update(self, t_now, **kwargs):
        return JoystickTeleopState(self.controller)


@dataclass(eq=False)
class JoystickHomingState(_ResumeToTeleop, ControllerState["ArmJoystickController"]):
    led_color = "magenta"

    def on_enter(self, from_state, **kwargs):
        self.controller.move_to(self.controller.arm_home, self.controller.homing_time)

    def update(self, t_now, **kwargs):
        if self.controller.arm.at_goal:
            return JoystickTeleopState(self.controller)
        return None

    def receive_inputs(self, t_now, inputs: ArmJoystickInputs, **kwargs):
        self.controller.set_gripper(inputs.gripper_closed)


@dataclass(eq=False)
class JoystickTeleopState(_ResumeToTeleop, ControllerState["ArmJoystickController"]):
    joint_target: NDArray[np.float64] = field(init=False)
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state, **kwargs):
        # A copy, never a view of the feedback buffer.
        self.joint_target = self.controller.current_position()
        self.controller.hold_position()

    def receive_inputs(self, t_now, inputs: ArmJoystickInputs, **kwargs):
        self.controller.set_gripper(inputs.gripper_closed)
        if self.requested is not None:
            return

        if inputs.home:
            self.requested = JoystickHomingState(self.controller)
            return

        if (target := self.compute_target(inputs)) is not None:
            self.joint_target = target
            self.controller.move_to(self.joint_target, self.controller.goal_time)

    def update(self, t_now, **kwargs):
        requested, self.requested = self.requested, None
        return requested

    def compute_target(self, inputs: ArmJoystickInputs) -> NDArray[np.float64] | None:
        controller = self.controller
        pos_curr = controller.current_position()

        xyz_curr = np.empty(3)
        rot_curr = np.empty((3, 3))
        try:
            controller.arm.FK(pos_curr, xyz_out=xyz_curr, orientation_out=rot_curr)
        except ValueError:
            print(f'{controller.prefix}Cannot compute FK at {pos_curr}, skipping sample')
            return None

        xyz_target = xyz_curr + inputs.delta_xyz

        r_x = R.from_euler('x', inputs.delta_rot_xyz[0])
        r_y = R.from_euler('y', inputs.delta_rot_xyz[1])
        r_z = R.from_euler('z', inputs.delta_rot_xyz[2])
        wrist_rot = R.from_euler('z', -pos_curr[5])
        rot_target = R.from_matrix(rot_curr) * wrist_rot * r_x * r_y * r_z * wrist_rot.inv()

        # Keep the IK seed on one side of the shoulder flip.
        seed = self.joint_target.copy()
        if seed[2] < controller.shoulder_flip_angle:
            seed[2] = controller.shoulder_flip_angle + abs(seed[2] - controller.shoulder_flip_angle)

        target = controller.arm.ik_target_xyz_so3(seed, xyz_target, rot_target.as_matrix())

        limits = controller.joint_limits
        if np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
            return None
        return target


class ArmJoystickController(ArmController[ArmJoystickInputs]):
    """Arm jogged in Cartesian space by joystick deltas.

    :param goal_time: Duration of each jog goal. Every input re-plans toward
        ``current pose + delta``, so this sets how aggressively the arm chases
        the joystick. Tune on hardware.
    """

    def __init__(
        self,
        arm: Arm,
        home_position: Sequence[float] | NDArray[np.float64],
        gripper: Gripper | None = None,
        *,
        homing_time: float = 5.0,
        goal_time: float = 0.5,
        shoulder_flip_angle: float = -np.pi / 2,
        joint_limits: NDArray[np.float64] | None = None,
        name: str = "Arm",
        input_timeout: float | None = 1.0,
    ):
        super().__init__(arm, JoystickStartupState, ArmFaultState, ArmExitState, gripper=gripper, name=name, input_timeout=input_timeout)
        self.arm_home = np.asarray(home_position, dtype=np.float64)
        self.homing_time = homing_time
        self.goal_time = goal_time
        # TODO: GENERALIZE THIS
        self.shoulder_flip_angle = shoulder_flip_angle
        if joint_limits is None:
            joint_limits = np.column_stack([np.full(arm.size, -np.inf), np.full(arm.size, np.inf)])
        self.joint_limits = np.asarray(joint_limits, dtype=np.float64)

    @classmethod
    def from_config(cls, arm: Arm, cfg: hebi.config.HebiConfig, gripper: Gripper | None = None, **kwargs):
        """Read ``home_position`` (required) and homing time from ``cfg.user_data``; keyword arguments override."""
        params = {
            "home_position": required_user_data(cfg, "home_position"),
            "homing_time": float(user_data_value(cfg, "homing_time", "homing_duration", default=5.0)),
        }
        params.update(kwargs)
        return cls(arm, gripper=gripper, **params)


def setup_mobile_io(m: MobileIO):
    m.resetUI()
    m.set_button_label(1, '⟲', blocking=False)
    m.set_button_label(2, '', blocking=False)
    m.set_button_label(3, '', blocking=False)
    m.set_button_label(4, 'Quit', blocking=False)
    m.set_button_label(6, '⇧', blocking=False)
    m.set_button_label(7, 'grip', blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, '⇩', blocking=False)

    m.set_axis_label(4, '', blocking=False)
    m.set_axis_label(5, '', blocking=False)
    m.set_axis_label(6, '', blocking=False)

    m.set_axis_label(1, '')
    m.set_axis_label(7, '')
    m.set_axis_label(2, 'rotate')
    m.set_axis_label(8, 'translate')
    m.set_axis_label(3, 'wrist', blocking=False)
    m.set_snap(3, 0)


def parse_mobile_feedback(m: MobileIO, timeout_ms: float = 100.0):
    if not m.update(timeout_ms):
        return None

    arm_dz = 0.0
    if m.get_button_state(6):
        arm_dz = 0.1
    elif m.get_button_state(8):
        arm_dz = -0.1

    return ArmJoystickInputs(
        home=bool(m.get_button_state(1)),
        delta_xyz=[0.25 * m.get_axis_state(8), -0.25 * m.get_axis_state(7), arm_dz],
        delta_rot_xyz=[0.5 * m.get_axis_state(1), -0.5 * m.get_axis_state(2), 0.75 * m.get_axis_state(3)],
        gripper_closed=bool(m.get_button_state(7)),
    )


if __name__ == "__main__":
    parser = config_arg_parser('Control a HEBI arm with joystick inputs', CONFIG_DIR)
    add_config_argument(parser, "config", "Arm config, e.g. A-2580-06G.cfg.yaml")
    parser.add_argument('--no-gripper', action='store_true', help="Don't use the gripper the config declares")
    args = parser.parse_args()
    cfg = load_config(args.config, CONFIG_DIR)

    lookup = hebi.Lookup()
    sleep(2)

    # Find all hardware before anything starts moving
    arm = hebi.arm.create_from_config(cfg, lookup)
    gripper = None if args.no_gripper else find_gripper(lookup, cfg)

    arm_control = ArmJoystickController.from_config(arm, cfg, gripper)

    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, cfg.families[0])
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, cfg.families[0])

    m.update()
    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    arm_control.start()
    try:
        while arm_control.running:
            arm_inputs = parse_mobile_feedback(m)
            if m.get_button_state(4):
                break
            if arm_inputs is not None:
                arm_control.receive_inputs(arm_inputs)
    except KeyboardInterrupt:
        pass
    finally:
        arm_control.stop()
        m.set_led_color('red')
