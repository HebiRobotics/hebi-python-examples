"""Drive a Tready / Treadward base and jog its arm with Mobile IO joysticks.

    python -m kits.tready.treaded_base_arm_joystick_control tready.cfg.yaml tready-arm-A-2240-06G.cfg.yaml

Drive layout: the right joystick and "Z" slider always translate the arm.
"Arm" toggles the left joystick and "Wrist" slider from driving the base to
rotating the arm's end effector. The reset button homes both base and arm.
"""

from __future__ import annotations

import typing

import hebi
import numpy as np

from kits.arms.joystick_control_sm import ArmJoystickController, ArmJoystickInputs

from .demo_runner import LAYOUT_DIR, create_arm, create_base_controller, load_config, parse_args, run
from .mobile_io_manager import MobileIOUpdater, parse_flippers, update_startup_mode
from .treaded_base import TreadyStartupState, TreadyTeleopState
from .treaded_base_core import ChassisVelocity, TreadyInputs
from .tready_utils import wait_for_mobile_io

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO

# TreadwardJoystickArmController.json
BUTTONS = {"reset": 1, "arm_mode": 3, "gripper": 4, "height_up": 5, "recenter": 6, "height_down": 7, "quit": 8}
AXES = {
    "turn": 1,  # left joystick, horizontal
    "forward": 2,  # left joystick, vertical
    "front_flippers": 3,
    "rear_flippers": 4,
    "wrist": 5,
    "z": 6,
    "arm_y": 7,  # right joystick, horizontal
    "arm_x": 8,  # right joystick, vertical
}

# Left joystick / wrist slider labels: (arm mode off, arm mode on)
ARM_MODE_LABELS = {AXES["turn"]: ("Turn", ""), AXES["forward"]: ("Drive", "Rotate"), AXES["wrist"]: ("", "Wrist")}


def update_arm_mode_labels(m: MobileIO):
    change = m.get_button_diff(BUTTONS["arm_mode"])
    if change == 0:
        return
    arm_mode = change == 1
    for axis, labels in ARM_MODE_LABELS.items():
        m.set_axis_label(axis, labels[arm_mode], blocking=False)


def parse_joystick_arm(m: MobileIO) -> ArmJoystickInputs:
    update_arm_mode_labels(m)

    # Axes can read NaN right after a layout is sent, before the phone reports them: treat that as no input
    def axis(name: str) -> float:
        return float(np.nan_to_num(m.get_axis_state(AXES[name]), nan=0.0, posinf=0.0, neginf=0.0))

    delta_xyz = [0.2 * axis("arm_x"), -0.2 * axis("arm_y"), 0.2 * axis("z")]
    if m.get_button_state(BUTTONS["arm_mode"]):
        delta_rot_xyz = [0.5 * axis("forward"), -0.5 * axis("turn"), 0.75 * axis("wrist")]
    else:
        delta_rot_xyz = [0.0, 0.0, 0.0]

    return ArmJoystickInputs(
        home=bool(m.get_button_state(BUTTONS["reset"])),
        delta_xyz=delta_xyz,
        delta_rot_xyz=delta_rot_xyz,
        gripper_closed=bool(m.get_button_state(BUTTONS["gripper"])),
    )


def parse_drive(m: MobileIO) -> TreadyInputs:
    if m.get_button_state(BUTTONS["quit"]):
        return TreadyInputs(quit=True)
    if m.get_button_state(BUTTONS["reset"]):
        return TreadyInputs(home=True)
    if m.get_button_state(BUTTONS["recenter"]):
        return TreadyInputs(align_flippers=True)

    sliders = (AXES["front_flippers"], AXES["front_flippers"], AXES["rear_flippers"], AXES["rear_flippers"])
    flippers = parse_flippers(m, BUTTONS["height_up"], BUTTONS["height_down"], sliders)
    if m.get_button_state(BUTTONS["arm_mode"]):
        # Left joystick is rotating the arm; don't drive.
        return TreadyInputs(flippers=flippers)
    return TreadyInputs(
        base_motion=ChassisVelocity(m.get_axis_state(AXES["forward"]), m.get_axis_state(AXES["turn"])),
        flippers=flippers,
    )


def parse_startup_mode(m: MobileIO):
    (base_inputs,) = update_startup_mode(m)
    return base_inputs, ArmJoystickInputs.idle()


def parse_drive_mode(m: MobileIO):
    return parse_drive(m), parse_joystick_arm(m)


if __name__ == "__main__":
    args = parse_args("Drive a treaded base and jog its arm with joysticks", arm_config="Arm config, e.g. tready-arm-A-2240-06G.cfg.yaml")
    base_cfg = load_config(args.base_config)
    arm_cfg = load_config(args.arm_config)
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving
    m = wait_for_mobile_io(lookup, base_cfg.families[0])
    print("mobileIO device found.")
    base_control = create_base_controller(base_cfg, lookup)
    arm, gripper = create_arm(arm_cfg, lookup)
    arm_control = ArmJoystickController.from_config(arm, arm_cfg, gripper)

    updater = MobileIOUpdater(
        m,
        {
            TreadyStartupState: (LAYOUT_DIR / "TreadwardStartupController.json", parse_startup_mode),
            TreadyTeleopState: (LAYOUT_DIR / "TreadwardJoystickArmController.json", parse_drive_mode),
        },
    )
    updater.attach(base_control)

    run(updater, [base_control, arm_control], log=not args.no_log)
