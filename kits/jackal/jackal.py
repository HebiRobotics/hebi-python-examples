#! /usr/bin/env python3

import os
import hebi
import numpy as np
from time import time, sleep
from scipy.spatial.transform import Rotation as R
from hebi.util import create_mobile_io

from util.lookup_utils import try_get_group_until_found

from ..camera.camera import HebiCamera
from ..camera.pan_tilt_mast import HebiCameraMast, MastControl, MastControlState, MastInputs
from ..arm.arm_ar_state_machine import ArmMobileIOControl, ArmControlState, ArmMobileIOInputs
from .jackal_control import JackalControl, JackalControlState, JackalInputs

import typing
if typing.TYPE_CHECKING:
    from typing import Sequence, Optional
    import numpy.typing as npt
    from hebi import Lookup
    from hebi._internal.mobile_io import MobileIO
    from hebi._internal.group import Group
    from hebi._internal.trajectory import Trajectory


def setup_arm_6dof(lookup: 'Lookup', family: str):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=os.path.join(root_dir, '../arm/hrdf/tready-arm-A2240-06G.hrdf'),
        lookup=lookup)

    arm.load_gains(os.path.join(root_dir, '../arm/gains/A-2240-06.xml'))

    # Add the gripper
    gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])
    while gripper_group is None:
        print(f'Looking for gripper module "{family}/gripperSpool"...')
        sleep(1)
        gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])

    gripper = hebi.arm.Gripper(gripper_group, -5, 1)
    gripper.load_gains(os.path.join(root_dir, '../arm/gains/gripper_spool_gains.xml'))
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

    m.set_button_label(1, '⟲')
    m.set_button_label(2, 'wide')
    m.set_button_mode(2, 1)
    m.set_button_label(3, 'flood')
    m.set_button_mode(3, 1)
    m.set_button_label(4, 'spot')
    m.set_button_mode(4, 1)
    m.set_button_label(5, 'arm')
    m.set_button_mode(5, 1)
    m.set_button_label(7, 'grip')
    m.set_button_mode(7, 1)


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None, None, None

    try:
        # reorder quaternion components
        wxyz = m.orientation
        xyzw = [*wxyz[1:4], wxyz[0]]
        rotation = R.from_quat(xyzw).as_matrix()
    except ValueError as e:
        print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
        rotation = np.eye(3)

    home = m.get_button_state(1)

    linear = m.get_axis_state(8)
    angular = m.get_axis_state(7)
    base_inputs = JackalInputs(linear, angular)

    arm_inputs = ArmMobileIOInputs(
        np.copy(m.position),
        rotation,
        home,
        m.get_button_state(5),
        m.get_button_state(7))

    flood_light = 0.0
    if m.get_button_state(3):
        flood_light = m.get_axis_state(4)

    spot_light = 0.0
    if m.get_button_state(4):
        spot_light = m.get_axis_state(4)

    mast_inputs = MastInputs(
        [m.get_axis_state(1), m.get_axis_state(2)],
        home,
        m.get_axis_state(3),
        flood_light,
        spot_light)

    return base_inputs, arm_inputs, mast_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Setup Camera Pan/Tilt
    family = "Mast"
    module_names = ['J1_pan', 'J2_tilt']

    arm = setup_arm_6dof(lookup, 'Arm')
    arm_control = ArmMobileIOControl(arm)

    group = try_get_group_until_found(lookup, family, module_names, 'Looking for pan/tilt modules...')
    mast = HebiCameraMast(group)
    zoom_group = try_get_group_until_found(lookup, 'C10', ['C10-0001'], 'Looking for zoom camera...')
    zoom_camera = HebiCamera(zoom_group)
    mast_control = MastControl(mast, zoom_camera)

    cam_group = try_get_group_until_found(lookup, 'CW1', ['CW1-0004'], 'Looking for camera...')
    camera = HebiCamera(cam_group)

    base_control = JackalControl()

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, family)
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, family)

    m.update()
    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    while base_control.running and arm_control.running and mast_control.running:
        t = time()
        try:
            base_inputs, arm_inputs, mast_inputs = parse_mobile_feedback(m)
            base_control.update(t, base_inputs)
            arm_control.update(t, arm_inputs)
            mast_control.update(t, mast_inputs)
        except KeyboardInterrupt:
            base_control.transition_to(t, JackalControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            mast_control.transition_to(t, MastControlState.EXIT)

        camera.update()
        # Update wide-angle camera flood light
        if m.get_button_state(2):
            camera.flood_light = m.get_axis_state(4)
        else:
            camera.flood_light = 0.0

        arm_control.send()
        mast_control.send()
        camera.send()
