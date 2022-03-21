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
from jackal.jackal_control import JackalControl, JackalControlState, JackalInputs

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
    m.set_button_label(8, '\u21E9')

    m.set_axis_label(3, '\U0001F50E', blocking=False)
    m.set_axis_value(3, -0.9)
    m.set_axis_label(4, '\U0001F4A1', blocking=False)
    m.set_axis_value(4, -0.9)


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None, None, None

    #try:
    #    # reorder quaternion components
    #    wxyz = m.orientation
    #    xyzw = [*wxyz[1:4], wxyz[0]]
    #    rotation = R.from_quat(xyzw).as_matrix()
    #except ValueError as e:
    #    print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
    #    rotation = np.eye(3)

    home = m.get_button_state(1)

    base_x = m.get_axis_state(8)
    base_rz = m.get_axis_state(7)

    if m.get_button_state(5):
        arm_dx = 0.3 * m.get_axis_state(8)
        arm_dy = -0.3 * m.get_axis_state(7)

        arm_dz = 0.0
        if m.get_button_state(6):
            arm_dz = 0.1
        elif m.get_button_state(8):
            arm_dz = -0.1

        arm_drx = 0.0
        arm_dry = 0.0
        arm_drz = 0.0
    else:
        arm_dx = 0.0
        arm_dy = 0.0
        arm_dz = 0.0

        arm_drx = 0.5 * m.get_axis_state(7)
        arm_dry = -0.5 * m.get_axis_state(8)
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

    mast_pan = -1.0 * m.get_axis_state(1)
    mast_tilt = m.get_axis_state(2)
    camera_zoom = m.get_axis_state(3)

    base_inputs = JackalInputs(base_x, base_rz)

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

    arm = setup_arm_6dof(lookup, 'Jackal')
    arm_control = ArmJoystickControl(arm, [0, -2, 1, 0, 0.5, 0], homing_time=7.0)

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

    cam_group = lookup.get_group_from_names('Jackal', ['CW1-0004'])
    while cam_group is None:
        print('Looking for camera...')
        sleep(1)
        cam_group = lookup.get_group_from_names('Jackal', ['CW1-0004'])
    camera = HebiCamera(cam_group)

    base_control = JackalControl()

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, 'Jackal')
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, 'Jackal')

    m.update()
    setup_mobile_io(m)

    def resend_mobile_io_setup(control: JackalControl, state: JackalControlState):
        if control.state == state.DISCONNECTED and state == state.TELEOP:
            setup_mobile_io(m)

    base_control._transition_handlers.append(resend_mobile_io_setup)

    #######################
    ## Main Control Loop ##
    #######################
    camera_angle_cmd = hebi.GroupCommand(1)

    while base_control.running and arm_control.running and mast_control.running:
        t = time()
        try:
            base_inputs, arm_inputs, mast_inputs = parse_mobile_feedback(m)
            base_control.update(t, None)
            arm_control.update(t, arm_inputs)
            mast_control.update(t, mast_inputs)
            camera.update()
            # Update mobileIO stream angle
            if mast_inputs:
                camera_angle_cmd.io.c.set_float(1, mast_control.camera.roll)
                m._group.send_command(camera_angle_cmd)
        except KeyboardInterrupt:
            base_control.transition_to(t, JackalControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            mast_control.transition_to(t, MastControlState.EXIT)

        # Update wide-angle camera flood light
        if m.get_button_state(2):
            camera.flood_light = (m.get_axis_state(4) + 1.0) / 2.0
        else:
            camera.flood_light = 0.0

        base_control.send()
        arm_control.send()
        mast_control.send()
        camera.send()
