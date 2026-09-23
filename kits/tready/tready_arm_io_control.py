from __future__ import annotations

import time
import datetime
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi

from kits.arms.ar_control_sm import ArmMobileIOControl, ArmMobileIOInputs

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


def update_ar_arm(m: MobileIO):
    # Drive buttons, joysticks, and sliders
    mapping = {
        "reset_pose_btn": 1,
        "arm_lock": 3,
        "gripper": 4,
    }

    try:
        # reorder quaternion components
        rotation = R.from_quat(m.orientation, scalar_first=True).as_matrix()
    except ValueError as e:
        print(f"Error getting orientation as matrix: {e}\n{m.orientation}")
        rotation = np.eye(3)

    arm_inputs = ArmMobileIOInputs(
        phone_pos=np.copy(m.position),
        phone_rot=rotation,
        lock_toggle=m.get_button_diff(mapping["arm_lock"]) != 0,
        locked=m.get_button_state(mapping["arm_lock"]) == 0,
        gripper_closed=m.get_button_state(mapping["gripper"]),
        home=m.get_button_state(mapping["reset_pose_btn"]),
    )

    return arm_inputs


def update_drive_mode(m: MobileIO):
    # Drive buttons, joysticks, and sliders
    mapping = {
        "reset_pose_btn": 1,
        "torque_btn": 2,
        "height_up_btn": 5,
        "recenter_btn": 6,
        "height_down_btn": 7,
        "quit_demo_btn": 8,
        "turn_joy": 1,
        "forward_joy": 2,
        "front_left_slider": 3,
        "front_right_slider": 4,
        "back_left_slider": 5,
        "back_right_slider": 6,
    }

    base_inputs = None

    if m.get_button_state(mapping["quit_demo_btn"]):
        base_inputs = TreadyInputs(quit=True)
    elif m.get_button_state(mapping["reset_pose_btn"]):
        base_inputs = TreadyInputs(
            home=True,
            torque_mode=m.get_button_state(mapping["torque_btn"]),
            torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
        )

    elif m.get_button_state(mapping["recenter_btn"]):
        base_inputs = TreadyInputs(
            align_flippers=True,
            torque_mode=m.get_button_state(mapping["torque_btn"]),
            torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
        )
    else:
        height_up = m.get_button_state(mapping["height_up_btn"])
        height_down = m.get_button_state(mapping["height_down_btn"])
        height = height_up - height_down
        if height != 0:
            flippers = [-height / 2] * 4
        else:
            flippers = [
                m.get_axis_state(mapping["front_left_slider"]),
                m.get_axis_state(mapping["front_right_slider"]),
                m.get_axis_state(mapping["back_left_slider"]),
                m.get_axis_state(mapping["back_right_slider"]),
            ]

        base_inputs = TreadyInputs(
            base_motion=ChassisVelocity(
                m.get_axis_state(mapping["forward_joy"]),
                m.get_axis_state(mapping["turn_joy"]),
            ),
            flippers=flippers,
            torque_mode=m.get_button_state(mapping["torque_btn"]),
            torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
        )

    if m.get_button_diff(mapping["torque_btn"]) == 1:
        axis_vals = [0, -0.5, 1, 1]
        for i in range(3, 7):
            if not m.set_snap(i, np.nan):
                print(f"Failed to set snap for axis {i}")
            if not m.set_axis_value(i, axis_vals[i - 3]):
                print(f"Failed to set axis value for axis {i}")

    elif m.get_button_diff(mapping["torque_btn"]) == -1:
        for i in range(3, 7):
            if not m.set_snap(i, 0):
                print(f"Failed to set snap for axis {i}")
        m.set_axis_label(mapping["front_left_slider"], "FL", blocking=False)
        m.set_axis_label(mapping["front_right_slider"], "FR", blocking=False)
        m.set_axis_label(mapping["back_left_slider"], "BL", blocking=False)
        m.set_axis_label(mapping["back_right_slider"], "BR", blocking=False)

    return base_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()

    config_dir = Path(__file__).absolute().parent / "config"
    base_cfg = hebi.config.load_config(str(config_dir / sys.argv[1]))

    def arm_update_startup_mode(m):
        return (update_startup_mode(m)[0], None)

    def arm_update_drive_mode(m):
        base_input = update_drive_mode(m)
        arm_input = update_ar_arm(m)
        return (base_input, arm_input)

    # mobileIO setup
    print("Looking for mobileIO device...")
    m = wait_for_mobile_io(lookup, base_cfg.families[0])

    print("mobileIO device found.")
    m.update()

    layout_dir = config_dir / "layouts"
    updater = MobileIOUpdater(
        m,
        {
            MobileIOModes.STARTUP: (layout_dir / "TreadwardStartupController.json", arm_update_startup_mode),
            MobileIOModes.DRIVE: (layout_dir / "TreadwardARArmController.json", arm_update_drive_mode),
        },
    )

    base = TreadedBase.from_config(base_cfg)
    base_control = TreadedBaseControl(base, max_base_speed=base_cfg.user_data["max_base_speed"])
    base_control.namespace = "[Base] "

    # Create Arm group
    arm_family = "Arm"
    config_file = config_dir / "tready-arm-A-2240-06G.cfg.yaml"
    cfg = hebi.config.load_config(str(config_file))
    arm = hebi.arm.create_from_config(cfg, lookup)
    gripper = None  # TODO: Reimplement the old behaviors for gripper loading
    # arm, gripper = setup_arm_6dof(lookup, arm_family, with_gripper=False)

    if arm is None:
        if len(cfg.families) == 1:
            families = [cfg.families[0]] * len(cfg.names)
        else:
            families = cfg.families
        pairs = zip(families, cfg.names)
        modules = [f"{p[0]}/{p[1]}" for p in pairs]
        raise RuntimeError(f"Could not find modules:\n{'\n'.join(modules)}")

    arm_control = ArmMobileIOControl(arm, gripper, traj_duration=0.5)
    arm_control.namespace = "[Arm] "
    arm_control.arm_ik_seed = np.array([0.3, 1.2, 2.2, 2.9, -1.57, 0])

    home_orientation = R.from_euler("z", np.pi / 2) * R.from_euler("x", np.pi)
    arm_control.set_arm_home(np.array([0.5, 0.0, 0.0]), home_orientation.as_matrix())

    base_control._transition_handlers.append(updater.base_transition_handler)

    base_control._update_handlers.append(updater.update_voltage_reading)
    base_control._update_handlers.append(updater.update_torque_mode)
    base_control._update_handlers.append(updater.update_startup_msg_base)

    # can enable start logging here
    logging = True

    if logging:
        tready_log_dir = str(Path(__file__) / "logs")
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

                if base_inputs.quit:
                    break

                base_control.receive_inputs(t, base_inputs)

            arm_control.update(t, arm_inputs)
            arm_control.send()

        except KeyboardInterrupt:
            break

    base_control.stop()
    arm_control.stop()

    if logging:
        base.group.stop_log()
        arm.group.stop_log()
