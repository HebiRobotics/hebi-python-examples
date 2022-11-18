#! /usr/bin/env python3

import hebi
import numpy as np
from time import time, sleep
from hebi.util import create_mobile_io

from kits.arm.joystick_control_sm import ArmJoystickControl, ArmControlState, ArmJoystickInputs
from .tready import TreadedBase, TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity
from .tready_utils import setup_arm_6dof, setup_arm_7dof

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def setup_mobile_io(m: 'MobileIO'):
    m.resetUI()
    m.set_button_label(1, '⟲', blocking=False)
    m.set_button_label(2, '', blocking=False)
    m.set_button_label(3, '', blocking=False)
    m.set_button_label(4, 'Quit', blocking=False)
    m.set_button_label(5, 'arm', blocking=False)
    m.set_button_mode(5, 1)
    m.set_button_label(6, '\u21E7', blocking=False)
    m.set_button_label(7, 'grip', blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, '\u21E9', blocking=False)

    m.set_axis_label(4, '', blocking=False)
    m.set_axis_label(5, 'front', blocking=False)
    m.set_snap(5, 0)
    m.set_axis_label(6, 'rear', blocking=False)
    m.set_snap(6, 0)

    m.set_axis_label(1, '')
    m.set_axis_label(7, '')
    if m.get_button_state(5):
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'translate')
        m.set_axis_label(3, 'wrist', blocking=False)
        m.set_snap(3, 0)
    else:
        m.set_axis_label(2, 'drive')
        m.set_axis_label(8, 'translate')
        m.set_axis_label(3, '', blocking=False)
        m.set_snap(3, np.nan)


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None, None

    home = m.get_button_state(1)

    if m.get_button_diff(5) == 1:
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'translate')
        m.set_axis_label(3, 'wrist', blocking=False)
        m.set_snap(3, 0)
    elif m.get_button_diff(5) == -1:
        m.set_axis_label(2, 'drive')
        m.set_axis_label(8, 'translate')
        m.set_axis_label(3, '', blocking=False)
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
        home,
        ChassisVelocity(base_x, base_rz),
        [flipper1, flipper1, flipper4, flipper4],
        True)

    arm_inputs = ArmJoystickInputs(
        home,
        [arm_dx, arm_dy, arm_dz],
        [arm_drx, arm_dry, arm_drz],
        gripper_closed=gripper_closed)

    return base_inputs, arm_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    arm = setup_arm_7dof(lookup, 'Arm')
    joint_limits = np.empty((7, 2))
    joint_limits[:, 0] = -np.inf
    joint_limits[:, 1] = np.inf

    # base limits [-2.6, 1.7] (radians)
    joint_limits[0, :] = [-2.6, 1.7]
    # shoulder limits [-2.25, -0.1]
    joint_limits[1, :] = [-2.25, -0.1]

    arm_control = ArmJoystickControl(arm,
                                     [0.0, -2.0, 3.14, -2.5, -1.57, -0.25, 0.0],
                                     homing_time=7.0,
                                     joint_limits=joint_limits)

    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # Create base group
    base_group = lookup.get_group_from_names('Tready', wheel_names + flipper_names)
    while base_group is None:
        print('Looking for Tready modules...')
        sleep(1)
        base_group = lookup.get_group_from_names('Tready', wheel_names + flipper_names)

    base = TreadedBase(base_group, 0.25, 0.33)
    base_control = TreadyControl(base)

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, 'Tready')
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, 'Tready')

    m.update()
    setup_mobile_io(m)

    def update_mobile_ui(controller: TreadyControl, new_state: TreadyControlState):
        if controller.state == TreadyControlState.DISCONNECTED and new_state == TreadyControlState.TELEOP:
            setup_mobile_io(m)

    base_control._transition_handlers.append(update_mobile_ui)

    #######################
    ## Main Control Loop ##
    #######################

    m.set_led_color('blue')
    while base_control.running and arm_control.running:
        t = time()
        try:
            base_inputs, arm_inputs = parse_mobile_feedback(m)
            base_control.update(t, base_inputs)
            arm_control.update(t, arm_inputs)
        except KeyboardInterrupt:
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')

        if m.get_button_state(4):
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')

        base_control.send()
        arm_control.send()
