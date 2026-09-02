from time import time, sleep
import datetime
from os.path import join, dirname, abspath

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

from kits.arms.ar_control_sm import ArmMobileIOControl, ArmMobileIOInputs

from .treaded_base_core import (
    TreadedBase,
    TreadedBaseConfig,
    TreadedBaseControl,
    TreadyInputs,
    ChassisVelocity,
)

from .mobile_io_manager import (
    MobileIOUpdater,
    MobileIOModes,
    update_startup_mode,
)

from .tready_utils import wait_for_mobile_io, try_create_base_group

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def update_inputs(base_inputs: "TreadyInputs | None" = None):
    if base_inputs is None:
        base_inputs = TreadyInputs()

    base_inputs.deploy_safe = True
    base_inputs.stow_safe = True
    base_inputs.payload_deployed = False
    base_inputs.allow_startup = True
    base_inputs.drive_safe = True

    return base_inputs


def arm_update_drive_mode(m: "MobileIO"):
    # Drive buttons, joysticks, and sliders
    mapping = {
        "reset_pose_btn": 1,
        "torque_btn": 2,
        "arm_lock": 3,
        "gripper": 4,
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
    arm_inputs = ArmMobileIOInputs()

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
        home=base_inputs.home,
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

    return (base_inputs, arm_inputs)


if __name__ == "__main__":
    config_dir = join(dirname(abspath(__file__)), "config")

    tready_config = TreadedBaseConfig(
        hrdf_file=join(config_dir, "hrdf", "Tready.hrdf"),
        gains_file=join(config_dir, "gains", "smart-tready-gains.xml"),
        wheel_diameter=0.125,  # m
        wheel_base=0.285,  # m
        torso_torque_scale=2.5,  # Nm
        torque_mode_max=25,  # Nm
    )

    layout_dir = join(config_dir, "layouts")
    startup_layout = join(layout_dir, "TreadwardStartupController.json")
    drive_layout = join(layout_dir, "TreadwardARArmController.json")

    def arm_update_startup_mode(m):
        return (update_startup_mode(m)[0], None)

    mio_demo_config = {
        MobileIOModes.STARTUP: (startup_layout, arm_update_startup_mode),
        MobileIOModes.DRIVE: (drive_layout, arm_update_drive_mode),
    }

    lookup = hebi.Lookup()
    sleep(2)

    base_family = "Tready"

    # mobileIO setup
    print("Looking for mobileIO device...")
    m = wait_for_mobile_io(lookup, base_family)

    print("mobileIO device found.")
    m.update()

    # Create base group
    base_group = try_create_base_group(lookup, base_family)
    while base_group is None:
        print(f"Looking for {base_family} modules...")
        sleep(1)
        base_group = try_create_base_group(lookup, base_family)

    base = TreadedBase(
        tready_config, base_group, chassis_ramp_time=0.5, flipper_ramp_time=0.1
    )
    base_control = TreadedBaseControl(base, max_base_speed=0.25)
    base_control.namespace = "[Base] "

    # Create Arm group
    arm_family = "Arm"
    config_file = join(config_dir, "tready-arm-A-2240-06G.cfg.yaml")
    cfg = hebi.config.load_config(config_file)
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

    arm_control = ArmMobileIOControl(arm, gripper)
    arm_control.namespace = "[Arm] "
    arm_control.arm_ik_seed = np.array([0.3, 1.2, 2.2, 2.9, -1.57, 0])

    home_orientation = R.from_euler("z", np.pi / 2) * R.from_euler("x", np.pi)
    arm_control.set_arm_home(np.array([0.5, 0.0, 0.0]), home_orientation.as_matrix())

    updater = MobileIOUpdater(m, mio_demo_config)
    base_control._transition_handlers.append(updater.base_transition_handler)

    base_control._update_handlers.append(updater.update_voltage_reading)
    base_control._update_handlers.append(updater.update_torque_mode)
    base_control._update_handlers.append(updater.update_startup_msg_base)

    # can enable start logging here
    logging = True

    if logging:
        tready_log_dir = join(dirname(__file__), "logs")
        now = datetime.datetime.now()
        arm.group.start_log(tready_log_dir, f"arm_{now:%Y-%m-%d-%H:%M:%S}")
        base.group.start_log(tready_log_dir, f"base_{now:%Y-%m-%d-%H:%M:%S}")

    while base_control.running and arm_control.running:
        t = time()
        try:
            inputs = updater.parse_mobile_io_feedback(m)
            if inputs is None:
                m_update = False
                base_inputs = update_inputs()
                arm_inputs = ArmMobileIOInputs()
            else:
                m_update = True
                base_inputs = update_inputs(inputs[0])
                arm_inputs = inputs[1]

            if base_inputs.quit:
                break

            base_control.update(t, m_update, base_inputs)
            arm_control.update(t, arm_inputs)
            base_control.send()
            arm_control.send()

        except KeyboardInterrupt:
            break

    base_control.stop()
    arm_control.stop()

    if logging:
        base.group.stop_log()
        arm.group.stop_log()
