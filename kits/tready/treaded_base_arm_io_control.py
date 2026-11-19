"""Drive a Tready / Treadward base and control its arm by moving the phone (AR).

    python -m kits.tready.treaded_base_arm_io_control tready.cfg.yaml tready-arm-A-2240-06G.cfg.yaml

Drive layout: left joystick drives, sliders/height buttons move the flippers,
"Arm" toggles phone tracking, "Gripper" toggles the gripper, and the reset
button homes both the base flippers and the arm.
"""

from __future__ import annotations

import typing

import hebi
import numpy as np

from kits.arms.ar_control_sm import ArmMobileIOController, ArmMobileIOInputs, phone_rotation

from .demo_runner import LAYOUT_DIR, create_arm, create_base_controller, load_config, parse_args, run
from .mobile_io_manager import MobileIOUpdater, handle_torque_toggle, parse_flippers, torque_inputs, update_startup_mode
from .treaded_base import TreadyStartupState, TreadyTeleopState
from .treaded_base_core import ChassisVelocity, TreadyInputs
from .tready_utils import wait_for_mobile_io

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO

# TreadwardARArmController.json
BUTTONS = {"reset": 1, "torque": 2, "arm_unlock": 3, "gripper": 4, "height_up": 5, "recenter": 6, "height_down": 7, "quit": 8}
AXES = {"turn": 1, "forward": 2}


def parse_ar_arm(m: MobileIO, tracking_enabled: bool = True) -> ArmMobileIOInputs:
    """Arm inputs from the phone pose. ``tracking_enabled=False`` keeps the arm locked (idle)."""
    return ArmMobileIOInputs(
        phone_pos=np.copy(m.position),
        phone_rot=phone_rotation(m),
        lock_toggle=tracking_enabled and m.get_button_diff(BUTTONS["arm_unlock"]) != 0,
        locked=not tracking_enabled or m.get_button_state(BUTTONS["arm_unlock"]) == 0,
        gripper_closed=tracking_enabled and bool(m.get_button_state(BUTTONS["gripper"])),
        home=tracking_enabled and bool(m.get_button_state(BUTTONS["reset"])),
    )


def parse_drive(m: MobileIO) -> TreadyInputs:
    # Buttons 3/4 belong to the arm here, so no rear-up/flatten as in the base demo.
    handle_torque_toggle(m, BUTTONS["torque"])
    torque = torque_inputs(m, BUTTONS["torque"])

    if m.get_button_state(BUTTONS["quit"]):
        return TreadyInputs(quit=True)
    if m.get_button_state(BUTTONS["reset"]):
        return TreadyInputs(home=True, **torque)
    if m.get_button_state(BUTTONS["recenter"]):
        return TreadyInputs(align_flippers=True, **torque)
    return TreadyInputs(
        base_motion=ChassisVelocity(m.get_axis_state(AXES["forward"]), m.get_axis_state(AXES["turn"])),
        flippers=parse_flippers(m, BUTTONS["height_up"], BUTTONS["height_down"]),
        **torque,
    )


def parse_startup_mode(m: MobileIO):
    (base_inputs,) = update_startup_mode(m)
    return base_inputs, parse_ar_arm(m, tracking_enabled=False)


def parse_drive_mode(m: MobileIO):
    return parse_drive(m), parse_ar_arm(m)


if __name__ == "__main__":
    args = parse_args(
        "Drive a treaded base and control its arm with phone AR tracking", arm_config="Arm config, e.g. tready-arm-A-2240-06G.cfg.yaml"
    )
    base_cfg = load_config(args.base_config)
    arm_cfg = load_config(args.arm_config)
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving
    m = wait_for_mobile_io(lookup, base_cfg.families[0])
    print("mobileIO device found.")
    base_control = create_base_controller(base_cfg, lookup)
    arm, gripper = create_arm(arm_cfg, lookup)
    arm_control = ArmMobileIOController.from_config(arm, arm_cfg, gripper)

    updater = MobileIOUpdater(
        m,
        {
            TreadyStartupState: (LAYOUT_DIR / "TreadwardStartupController.json", parse_startup_mode),
            TreadyTeleopState: (LAYOUT_DIR / "TreadwardARArmController.json", parse_drive_mode),
        },
    )
    updater.attach(base_control)

    run(updater, [base_control, arm_control], log=not args.no_log)
