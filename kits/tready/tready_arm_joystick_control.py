#! /usr/bin/env python3

import os
from os.path import join, dirname, abspath
import hebi
import numpy as np
from time import time, sleep
import datetime

from kits.arms.joystick_control_sm import (
    ArmJoystickControl,
    ArmControlState,
    ArmJoystickInputs,
)
from .treaded_base_core import (
    TreadedBase,
    TreadedBaseConfig,
    TreadedBaseControl,
    TreadyControlState,
    TreadyInputs,
    ChassisVelocity,
)
from .tready_utils import (
    setup_arm_6dof,
    setup_arm_7dof,
    wait_for_mobile_io,
    try_create_base_group,
)

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def setup_mobile_io(m: "MobileIO"):
    m.resetUI()
    m.set_button_label(1, "⟲", blocking=False)
    m.set_button_label(2, "", blocking=False)
    m.set_button_label(3, "", blocking=False)
    m.set_button_label(4, "Quit", blocking=False)
    m.set_button_label(5, "arm", blocking=False)
    m.set_button_mode(5, 1)
    m.set_button_label(6, "\u21e7", blocking=False)
    m.set_button_label(7, "grip", blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, "\u21e9", blocking=False)

    m.set_axis_label(4, "", blocking=False)
    m.set_axis_label(5, "front", blocking=False)
    m.set_snap(5, 0)
    m.set_axis_label(6, "rear", blocking=False)
    m.set_snap(6, 0)

    m.set_axis_label(1, "")
    m.set_axis_label(7, "")
    if m.get_button_state(5):
        m.set_axis_label(2, "rotate")
        m.set_axis_label(8, "translate")
        m.set_axis_label(3, "wrist", blocking=False)
        m.set_snap(3, 0)
    else:
        m.set_axis_label(2, "drive")
        m.set_axis_label(8, "translate")
        m.set_axis_label(3, "", blocking=False)
        m.set_snap(3, np.nan)


def parse_mobile_feedback(m: "MobileIO"):
    if not m.update(0.0):
        return None, None

    home = m.get_button_state(1)

    if m.get_button_diff(5) == 1:
        m.set_axis_label(2, "rotate")
        m.set_axis_label(8, "translate")
        m.set_axis_label(3, "wrist", blocking=False)
        m.set_snap(3, 0)
    elif m.get_button_diff(5) == -1:
        m.set_axis_label(2, "drive")
        m.set_axis_label(8, "translate")
        m.set_axis_label(3, "", blocking=False)
        m.set_snap(3, np.nan)

    arm_dx = 0.25 * m.get_axis_state(8)
    arm_dy = -0.25 * m.get_axis_state(7)

    arm_dz = 0.0
    if m.get_button_state(6):
        arm_dz = 0.1
    elif m.get_button_state(8):
        arm_dz = -0.1

    if m.get_button_state(5):
        base_x = 0.0
        base_rz = 0.0

        arm_drx = 0.5 * m.get_axis_state(1)
        arm_dry = -0.5 * m.get_axis_state(2)
        arm_drz = 0.75 * m.get_axis_state(3)

    else:
        base_x = m.get_axis_state(2)
        base_rz = m.get_axis_state(1) * 2.0

        arm_drx = 0.0
        arm_dry = 0.0
        arm_drz = 0.0

    gripper_closed = m.get_button_state(7)

    flipper1 = m.get_axis_state(5)
    flipper4 = m.get_axis_state(6)

    base_inputs = TreadyInputs(
        home=home,
        base_motion=ChassisVelocity(base_x, base_rz),
        flippers=[flipper1, flipper1, flipper4, flipper4],
        align_flippers=True,
    )

    arm_inputs = ArmJoystickInputs(
        home,
        [arm_dx, arm_dy, arm_dz],
        [arm_drx, arm_dry, arm_drz],
        gripper_closed=gripper_closed,
    )

    return base_inputs, arm_inputs


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
    lookup = hebi.Lookup()
    sleep(2)

    # Setup MobileIO
    m = wait_for_mobile_io(lookup, "Tready")

    m.update()
    setup_mobile_io(m)

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

    home_pose = np.array([0.3, 1.2, 2.2, 2.9, -1.57, 0])
    arm_control = ArmJoystickControl(
        arm,
        home_pose,
        homing_time=7.0,
    )
    arm_control.namespace = "[Arm] "

    base_family = "Tready"
    base_group = try_create_base_group(lookup, base_family)
    while base_group is None:
        print(f"Looking for {base_family} modules...")
        sleep(1)
        base_group = try_create_base_group(lookup, base_family)

    base = TreadedBase(tready_config, base_group, 0.25, 0.33)
    base_control = TreadedBaseControl(base, max_base_speed=0.25)
    base_control.namespace = "[Base] "

    def update_mobile_ui(controller: TreadedBaseControl, new_state: TreadyControlState):
        if (
            controller.state == TreadyControlState.DISCONNECTED
            and new_state == TreadyControlState.TELEOP
        ):
            setup_mobile_io(m)

    base_control._transition_handlers.append(update_mobile_ui)

    #######################
    ## Main Control Loop ##
    #######################

    logging = True

    if logging:
        tready_dir = os.path.dirname(__file__)
        now = datetime.datetime.now()
        arm.group.start_log(
            os.path.join(tready_dir, "logs"), f"arm_{now:%Y-%m-%d-%H:%M:%S}"
        )
        base.group.start_log(
            os.path.join(tready_dir, "logs"), f"base_{now:%Y-%m-%d-%H:%M:%S}"
        )

    m.set_led_color("blue")
    while base_control.running and arm_control.running:
        t = time()
        try:
            base_inputs, arm_inputs = parse_mobile_feedback(m)
            base_control.update(t, base_inputs)
            arm_control.update(t, arm_inputs)
        except KeyboardInterrupt:
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color("red")

        if m.get_button_state(4):
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color("red")

        base_control.send()
        arm_control.send()

    if logging:
        arm.group.stop_log()
        base.group.stop_log()
