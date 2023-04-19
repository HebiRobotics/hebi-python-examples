#! /usr/bin/env python3

import os
import hebi
import numpy as np
from time import time, sleep
from hebi.util import create_mobile_io

from kits.camera.camera import HebiCamera
from kits.arm.joystick_control_sm import ArmJoystickControl, ArmControlState, ArmJoystickInputs
from .tready import TreadedBase, TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity
from .tready_utils import setup_arm_6dof, setup_arm_7dof

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def setup_mobile_io(m: 'MobileIO'):
    m.resetUI()
    m.set_button_label(1, '⟲', blocking=False)
    m.set_button_label(2, 'flood', blocking=False)
    m.set_button_mode(2, 1)
    m.set_button_label(3, 'Quit', blocking=False)
    m.set_button_label(4, 'spot', blocking=False)
    m.set_button_mode(4, 1)
    m.set_button_label(5, 'arm', blocking=False)
    m.set_button_mode(5, 1)
    m.set_button_label(6, '\u21E7', blocking=False)
    m.set_button_label(7, '', blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, '\u21E9', blocking=False)

    m.set_axis_label(1, '')
    m.set_axis_label(3, 'zoom', blocking=False)
    m.set_axis_label(4, '', blocking=False)
    m.set_axis_label(5, 'front', blocking=False)
    m.set_snap(5, 0)
    m.set_axis_label(6, 'rear', blocking=False)
    m.set_snap(6, 0)
    m.set_axis_label(7, '')

    if m.get_button_state(5):
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'translate')
    else:
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'drive')


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None, None

    home = m.get_button_state(1)

    if m.get_button_diff(5) == 1:
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'translate')
    elif m.get_button_diff(5) == -1:
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'drive')

    arm_drz = 0.0
    arm_drx = 0.5 * m.get_axis_state(2)
    arm_dry = 0.5 * m.get_axis_state(1)

    arm_dz = 0.0
    if m.get_button_state(6):
        arm_dz = 0.1
    elif m.get_button_state(8):
        arm_dz = -0.1

    if m.get_button_state(5):
        base_x = 0.0
        base_rz = 0.0

        arm_dx = 0.25 * m.get_axis_state(8)
        arm_dy = -0.25 * m.get_axis_state(7)

    else:
        base_x = m.get_axis_state(8)
        base_rz = m.get_axis_state(7) * 2.0

        arm_dx = 0.0
        arm_dy = 0.0

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
        gripper_closed=False)

    return base_inputs, arm_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    hrdf_file = 'hrdf/A-2240-06C.hrdf'
    gains_file = 'gains/A-2240-06.xml'

    filename = os.path.abspath(__file__)
    root_dir = filename.split('kits')[0]

    hrdf_file = os.path.join(root_dir, 'kits/arm', hrdf_file)
    gains_file = os.path.join(root_dir, 'kits/arm', gains_file)

    arm = hebi.arm.create(
        ['Arm'],
        ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=hrdf_file,
        lookup=lookup)

    arm.load_gains(gains_file)

    joint_limits = np.empty((6, 2))
    joint_limits[:, 0] = -np.inf
    joint_limits[:, 1] = np.inf

    # base limits [-2, 2] (radians)
    joint_limits[0, :] = [np.pi / 2.0, 3.0 / 2.0 * np.pi]
    # shoulder limits [-2, inf]
    joint_limits[1, 0] = 0.5
    joint_limits[1, 1] = 1.3

    joints_home = [np.pi, 0.7, -2.40, np.pi / 2, np.pi / 2, np.pi / 2]

    arm_control = ArmJoystickControl(arm,
                                     joints_home,
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

    zoom_group = lookup.get_group_from_names('C10', ['C10-0003'])
    while zoom_group is None:
        print('Looking for zoom camera...')
        sleep(1)
        zoom_group = lookup.get_group_from_names('C10', ['C10-0003'])

    zoom_camera = HebiCamera(zoom_group)

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

    roll_cmd = hebi.GroupCommand(1)

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
            zoom_camera.update()

            if base_inputs is not None:
                if arm_control.at_limit:
                    m.set_led_color('yellow', blocking=False)
                else:
                    m.set_led_color('blue', blocking=False)

                roll_cmd.io.c.set_float(1, zoom_camera.roll)
                m._group.send_command(roll_cmd)

                zoom_camera.zoom_level = (m.get_axis_state(3) + 1.0) / 2

            if m.get_button_state(2):
                zoom_camera.flood_light = 0.5
            else:
                zoom_camera.flood_light = 0.0

            if m.get_button_state(4):
                zoom_camera.spot_light = 0.5
            else:
                zoom_camera.spot_light = 0.0

        except KeyboardInterrupt:
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')

        if m.get_button_state(3):
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')

        base_control.send()
        arm_control.send()
        zoom_camera.send()
