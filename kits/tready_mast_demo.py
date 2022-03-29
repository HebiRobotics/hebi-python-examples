#! /usr/bin/env python3

import os
import hebi
import numpy as np
from time import time, sleep
from scipy.spatial.transform import Rotation as R
from hebi.util import create_mobile_io

from camera.camera import HebiCamera
from camera.pan_tilt_mast import HebiCameraMast, MastControl, MastControlState, MastInputs
from arm.arm_ar_state_machine import ArmJoystickControl, ArmControlState, ArmJoystickInputs
#from jackal.jackal_control import JackalControl, JackalControlState, JackalInputs
from tready.tready import TreadedBase, TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity

import typing
if typing.TYPE_CHECKING:
    from hebi import Lookup
    from hebi._internal.mobile_io import MobileIO


# 192.168.131.116 : zoom
# 192.168.131.117 : wide angle


def setup_arm_6dof(lookup: 'Lookup', family: str):
    root_dir = os.path.abspath(os.path.dirname(__file__))
    print(root_dir)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2A_shoulder', 'J3_elbow1', 'J4_elbow2', 'J5_wrist1', 'J6_wrist2'],
        hrdf_file=os.path.join(root_dir, 'arm/hrdf/A-2303-06G.hrdf'),
        lookup=lookup)

    mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder'])
    while mirror_group is None:
        print(f'Looking for double shoulder module "{family}/J2B_shoulder"...')
        sleep(1)
        mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder'])

    arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    arm.load_gains(os.path.join(root_dir, 'arm/gains/A-2303-06.xml'))

    # Add the gripper
    gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])
    while gripper_group is None:
        print(f'Looking for gripper module "{family}/gripperSpool"...')
        sleep(1)
        gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])

    gripper = hebi.arm.Gripper(gripper_group, -5, 1)
    gripper.load_gains(os.path.join(root_dir, 'arm/gains/gripper_spool_gains.xml'))
    arm.set_end_effector(gripper)

    return arm


def setup_mobile_io(m: 'MobileIO'):
    # Button Assignments
    # 1 (Momentary): Home/Reset
    # 2 (Toggle): Wide Angle Camera Floodlight On/Off
    # 3 (Toggle): Zoom Camera Floodlight On/Off
    # 4 (Toggle): Zoom Camera Spotlight On/Off
    # 5 (Toggle): Arm Enable
    # 6 (Momentary):
    # 7 (Toggle): Gripper Control
    # 8 (Momentary):

    # Axis Assignments
    # 1: Camera Mast Pan
    # 2: Camera Mast Tilt
    # 3: Camera Mast Zoom
    # 4: Camera Light Level
    # 5:
    # 6:
    # 7: Jackal Fwd/Back
    # 8: Jackal Left/Right

    m.set_button_label(1, '⟲', blocking=False)
    m.set_button_label(2, 'wide', blocking=False)
    m.set_button_mode(2, 1)
    m.set_button_label(3, 'flood', blocking=False)
    m.set_button_mode(3, 1)
    m.set_button_label(4, 'spot', blocking=False)
    m.set_button_mode(4, 1)
    m.set_button_label(5, 'arm', blocking=False)
    m.set_button_mode(5, 1)
    m.set_button_label(6, '\u21E7', blocking=False)
    m.set_button_label(7, 'grip', blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, '\u21E9', blocking=False)

    m.set_axis_label(3, '\U0001F50E', blocking=False)
    m.set_axis_value(3, -0.9)
    m.set_axis_label(4, '\U0001F4A1', blocking=False)
    m.set_axis_value(4, -0.9)
    m.set_axis_label(5, 'front', blocking=False)
    m.set_snap(5, 0)
    m.set_axis_label(6, 'rear', blocking=False)
    m.set_snap(6, 0)

    m.set_axis_label(1, '')
    m.set_axis_label(7, '')
    if m.get_button_state(5):
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'translate')
    else:
        m.set_axis_label(2, 'pan/tilt')
        m.set_axis_label(8, 'drive')


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None, None, None

    home = m.get_button_state(1)

    if m.get_button_diff(5) == 1:
        m.set_axis_label(2, 'rotate')
        m.set_axis_label(8, 'translate')
    elif m.get_button_diff(5) == -1:
        m.set_axis_label(2, 'pan/tilt')
        m.set_axis_label(8, 'drive')

    if m.get_button_state(5):
        base_x = 0.0
        base_rz = 0.0
        mast_pan = 0.0
        mast_tilt = 0.0

        arm_dx =  0.3 * m.get_axis_state(8)
        arm_dy = -0.3 * m.get_axis_state(7)

        arm_dz = 0.0
        if m.get_button_state(6):
            arm_dz = 0.1
        elif m.get_button_state(8):
            arm_dz = -0.1

        arm_drx =  0.5 * m.get_axis_state(1)
        arm_dry = -0.5 * m.get_axis_state(2)
        arm_drz = 0.0
    else:
        mast_pan = -1.0 * m.get_axis_state(1)
        mast_tilt = m.get_axis_state(2)

        base_x = m.get_axis_state(8)
        base_rz = m.get_axis_state(7) * 2.0

        arm_dx = 0.0
        arm_dy = 0.0
        arm_dz = 0.0

        arm_drx = 0.0
        arm_dry = 0.0
        arm_drz = 0.0

    gripper_closed = m.get_button_state(7)

    # rescale slider to range [0, 1]
    light_level = (m.get_axis_state(4) + 1.0) / 2.0
    flood_light = 0.0
    if m.get_button_state(3):
        flood_light = light_level

    spot_light = 0.0
    if m.get_button_state(4):
        spot_light = light_level

    camera_zoom = m.get_axis_state(3)
    
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

    mast_inputs = MastInputs(
        [mast_pan, mast_tilt],
        home,
        camera_zoom,
        flood_light,
        spot_light)

    return base_inputs, arm_inputs, mast_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Setup Camera Pan/Tilt
    family = "Mast"
    module_names = ['J1_pan', 'J2_tilt']

    arm = setup_arm_6dof(lookup, 'Tready')
    joint_limits = np.empty((6, 2))
    joint_limits[:, 0] = -np.inf
    joint_limits[:, 1] = np.inf

    # base limits [-2, 2] (radians)
    joint_limits[0, :] = [-2.0, 2.0]
    # shoulder limits [-2, inf]
    joint_limits[1, 0] = -2.0

    arm_control = ArmJoystickControl(arm, [-0.5, -2, 1, 0, 0.5, -0.5], homing_time=7.0, joint_limits=joint_limits)

    group = lookup.get_group_from_names(family, module_names)
    while group is None:
        print('Looking for pan/tilt modules...')
        sleep(1)
        group = lookup.get_group_from_names(family, module_names)
    mast = HebiCameraMast(group)

    zoom_group = lookup.get_group_from_names('Mast', ['C10-0001'])
    while zoom_group is None:
        print('Looking for zoom camera...')
        sleep(1)
        zoom_group = lookup.get_group_from_names('Mast', ['C10-0001'])
    zoom_camera = HebiCamera(zoom_group)
    mast_control = MastControl(mast, zoom_camera)

    cam_group = lookup.get_group_from_names('Tready', ['CW1-0004'])
    while cam_group is None:
        print('Looking for camera...')
        sleep(1)
        cam_group = lookup.get_group_from_names('Tready', ['CW1-0004'])
    camera = HebiCamera(cam_group)

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
    camera_angle_cmd = hebi.GroupCommand(1)

    m.set_led_color('blue')
    while base_control.running and arm_control.running and mast_control.running:
        t = time()
        try:
            base_inputs, arm_inputs, mast_inputs = parse_mobile_feedback(m)
            base_control.update(t, base_inputs)
            arm_control.update(t, arm_inputs)
            mast_control.update(t, mast_inputs)
            camera.update()
            # Update mobileIO stream angle
            if mast_inputs:
                camera_angle_cmd.io.c.set_float(1, mast_control.camera.roll)
                camera_angle_cmd.io.c.set_float(2, 0.0)
                m._group.send_command(camera_angle_cmd)
        except KeyboardInterrupt:
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            mast_control.transition_to(t, MastControlState.EXIT)
            m.set_led_color('red')

        # Update wide-angle camera flood light
        if m.get_button_state(2):
            camera.flood_light = (m.get_axis_state(4) + 1.0) / 2.0
        else:
            camera.flood_light = 0.0

        base_control.send()
        arm_control.send()
        mast_control.send()
        camera.send()
