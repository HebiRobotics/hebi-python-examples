from __future__ import annotations

import time
import datetime
import sys
from pathlib import Path

import numpy as np
import hebi

from kits.arms.joystick_control_sm import (
    ArmJoystickControl,
    ArmJoystickInputs,
)
from .treaded_base_core import ChassisVelocity, TreadyInputs
from .treaded_base import (
    TreadedBase,
    TreadedBaseControl,
)

from .mobile_io_manager import (
    MobileIOUpdater,
    MobileIOModes,
    update_startup_mode,
)

from .tready_utils import wait_for_mobile_io


import typing

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def update_arm_labels(m: MobileIO):
    mapping = {
        "arm_layer": 3,
        "right_joystick": 2,
        "wrist_slider": 5,
    }
    layer_change = m.get_button_diff(mapping["arm_layer"])

    if layer_change == 1:
        m.set_axis_label(mapping["right_joystick"], "rotate")
        m.set_axis_label(mapping["wrist_slider"], "wrist", blocking=False)
        m.set_snap(mapping["wrist_slider"], 0)
    elif layer_change == -1:
        m.set_axis_label(mapping["right_joystick"], "drive")
        m.set_axis_label(mapping["wrist_slider"], "", blocking=False)
        m.set_snap(mapping["wrist_slider"], np.nan)


def update_joystick_arm(m: MobileIO):
    # Drive buttons, joysticks, and sliders
    mapping = {
        "reset_pose_btn": 1,
        "arm_mode": 3,
        "gripper": 4,
        "arm_drx_slider": 2,
        "arm_dry_slider": 1,
        "arm_drz_slider": 5,
        "arm_dx_slider": 8,
        "arm_dy_slider": 7,
        "arm_dz_slider": 6,
    }

    update_arm_labels(m)

    arm_dx = 0.2 * m.get_axis_state(mapping["arm_dx_slider"])
    arm_dy = -0.2 * m.get_axis_state(mapping["arm_dy_slider"])
    arm_dz = 0.2 * m.get_axis_state(mapping["arm_dz_slider"])

    if m.get_button_state(mapping["arm_mode"]):
        arm_drx = 0.5 * m.get_axis_state(mapping["arm_drx_slider"])
        arm_dry = -0.5 * m.get_axis_state(mapping["arm_dry_slider"])
        arm_drz = 0.75 * m.get_axis_state(mapping["arm_drz_slider"])

    else:
        arm_drx = 0.0
        arm_dry = 0.0
        arm_drz = 0.0

    arm_inputs = ArmJoystickInputs(
        home=m.get_button_state(mapping["reset_pose_btn"]),
        delta_xyz=[arm_dx, arm_dy, arm_dz],
        delta_rot_xyz=[arm_drx, arm_dry, arm_drz],
        gripper_closed=m.get_button_state(mapping["gripper"]),
    )

    return arm_inputs


def update_drive_mode(m: MobileIO):
    # Drive buttons, joysticks, and sliders
    mapping = {
        "reset_pose_btn": 1,
        "arm_mode": 3,
        "height_up_btn": 5,
        "recenter_btn": 6,
        "height_down_btn": 7,
        "quit_demo_btn": 8,
        "turn_joy": 1,
        "forward_joy": 2,
        "front_slider": 3,
        "back_slider": 4,
    }

    base_inputs = None

    if m.get_button_state(mapping["quit_demo_btn"]):
        base_inputs = TreadyInputs(quit=True)
    elif m.get_button_state(mapping["reset_pose_btn"]):
        base_inputs = TreadyInputs(
            home=True,
        )

    elif m.get_button_state(mapping["recenter_btn"]):
        base_inputs = TreadyInputs(
            align_flippers=True,
        )
    else:
        height_up = m.get_button_state(mapping["height_up_btn"])
        height_down = m.get_button_state(mapping["height_down_btn"])
        height = height_up - height_down
        if height != 0:
            flippers = [-height / 2] * 4
        else:
            flippers = [
                m.get_axis_state(mapping["front_slider"]),
                m.get_axis_state(mapping["front_slider"]),
                m.get_axis_state(mapping["back_slider"]),
                m.get_axis_state(mapping["back_slider"]),
            ]

        if m.get_button_state(mapping["arm_mode"]) == 0:
            base_inputs = TreadyInputs(
                base_motion=ChassisVelocity(
                    m.get_axis_state(mapping["forward_joy"]),
                    m.get_axis_state(mapping["turn_joy"]),
                ),
                flippers=flippers,
            )
        else:
            base_inputs = TreadyInputs(
                flippers=flippers,
            )

    return base_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()

    config_dir = Path(__file__).absolute().parent / "config"
    base_cfg = hebi.config.load_config(str(config_dir / sys.argv[1]))

    def arm_update_startup_mode(m):
        return (update_startup_mode(m)[0], None)

    def arm_update_drive_mode(m):
        base_input = update_drive_mode(m)
        arm_input = update_joystick_arm(m)
        return (base_input, arm_input)

    base_family = "Tready"

    # mobileIO setup
    print("Looking for mobileIO device...")
    m = wait_for_mobile_io(lookup, base_family)

    print("mobileIO device found.")
    m.update()

    layout_dir = config_dir / "layouts"
    mio_demo_config = {
        MobileIOModes.STARTUP: (layout_dir / "TreadwardStartupController.json", arm_update_startup_mode),
        MobileIOModes.DRIVE: (layout_dir / "TreadwardJoystickArmController.json", arm_update_drive_mode),
    }
    updater = MobileIOUpdater(m, mio_demo_config)

    base = TreadedBase.from_config(base_cfg)
    base_control = TreadedBaseControl(base, max_base_speed=base_cfg.user_data["max_base_speed"])
    base_control.namespace = "[Base] "

    # Create Arm group
    arm_family = "Arm"
    config_file = config_dir / "tready-arm-A-2240-06G.cfg.yaml"
    arm_cfg = hebi.config.load_config(str(config_file))
    arm = hebi.arm.create_from_config(arm_cfg, lookup)
    gripper = None  # TODO: Reimplement the old behaviors for gripper loading
    # arm, gripper = setup_arm_6dof(lookup, arm_family, with_gripper=False)

    if arm is None:
        if len(arm_cfg.families) == 1:
            families = [arm_cfg.families[0]] * len(arm_cfg.names)
        else:
            families = arm_cfg.families
        pairs = zip(families, arm_cfg.names)
        modules = [f"{p[0]}/{p[1]}" for p in pairs]
        raise RuntimeError(f"Could not find modules:\n{'\n'.join(modules)}")

    home_pose = np.array([0.3, 1.2, 2.2, 2.9, -1.57, 0])
    arm_control = ArmJoystickControl(
        arm,
        home_pose,
        homing_time=7.0,
    )
    arm_control.namespace = "[Arm] "

    base_control._transition_handlers.append(updater.base_transition_handler)

    base_control._update_handlers.append(updater.update_voltage_reading)
    base_control._update_handlers.append(updater.update_startup_msg_base)

    # can enable start logging here
    logging = True

    if logging:
        tready_log_dir = str(Path(__file__).parent /  "logs")
        now = datetime.datetime.now()
        arm.group.start_log(tready_log_dir, f"arm_{now:%Y-%m-%d-%H:%M:%S}")
        base.group.start_log(tready_log_dir, f"base_{now:%Y-%m-%d-%H:%M:%S}")

    while base_control.running and arm_control.running:
        t = time.time()
        try:
            inputs = updater.parse_mobile_io_feedback(m)
            if inputs is None:
                base_inputs = None
                arm_inputs = None
            else:
                base_inputs = inputs[0]
                arm_inputs = inputs[1]

                base_control.receive_inputs(t, base_inputs)
                if base_inputs.quit:
                    break

            arm_control.update(t, arm_inputs)
            arm_control.send()

        except KeyboardInterrupt:
            break

    base_control.stop()
    arm_control.stop()

    if logging:
        arm.group.stop_log()
