#!/usr/bin/env python3

import os
from time import time, sleep
import numpy as np

import hebi
from hebi.util import create_mobile_io
from hebi.arm import Gripper

from gripper_control import GripperControl, GripperInputs

from MAPS_input_device_example import ContinuousAngleMaps, LeaderFollowerControl, LeaderFollowerControlState, LeaderFollowerInputs

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None, None

    if m.get_button_diff(2) == 1:
        gripper_target = 1.0
        m.set_axis_value(3, 1.0)
    elif m.get_button_diff(4) == 1:
        gripper_target = 0.0
        m.set_axis_value(3, -1.0)
    else:
        # rescale to range [0, 1]
        gripper_target = (m.get_axis_state(3) + 1.0) / 2.0

    reset_arm = False
    if m.get_button_diff(8) == 1:
        reset_arm = True

    # Build an input object using the Mobile IO state
    return LeaderFollowerInputs(reset_arm), GripperInputs(gripper_target)


def setup_mobile_io(m: 'MobileIO'):
    m.resetUI()
    for i in range(8):
        m.set_button_label(i + 1, '')
        m.set_axis_label(i + 1, '')

    m.set_button_label(2, 'close', blocking=False)
    m.set_button_mode(2, 0)
    m.set_button_label(4, 'open', blocking=False)
    m.set_button_mode(4, 0)
    m.set_axis_label(3, 'grip', blocking=False)
    m.set_axis_value(3, -1.0)
    m.set_button_label(8, 'Reset', blocking=False)


if __name__ == "__main__":

    base_dir, _ = os.path.split(__file__)

    lookup = hebi.Lookup()
    sleep(2)

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, 'Arm')
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, 'Arm')

    setup_mobile_io(m)

    maps_modules = ['J1-A', 'J2-B', 'J2-A', 'J3-B', 'J3-A', 'J4-B', 'J4-A']
    maps_group = lookup.get_group_from_names(['MAPS'], maps_modules)
    while maps_group is None:
        m.clear_text(blocking=False)
        m.add_text('MAPS arm not found: Check connection and make sure all modules are blinking green', blocking=False)
        sleep(1)
        maps_group = lookup.get_group_from_names(['MAPS'], maps_modules)

    # need these b/c MAPS joint zeros are in different locations
    angle_offsets = np.array([0.0, np.pi / 2, -np.pi, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
    input_arm = ContinuousAngleMaps(maps_group, angle_offsets)
    output_arm = hebi.arm.create(
        ['Arm'],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2'],
        hrdf_file=os.path.join(base_dir, 'hrdf/A-2303-01.hrdf'),
        lookup=lookup)

    mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    while mirror_group is None:
        m.clear_text(blocking=False)
        m.add_text('Still looking for mirror group...', blocking=False)
        sleep(1)
        mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    # mirror the position/velocity/effort of module 1 ('J2A_shoulder1') to the module
    # in the mirror group ('J2B_shoulder1')
    # Keeps the two modules in the double shoulder bracket in sync
    output_arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    output_arm.load_gains(os.path.join(base_dir, 'gains/A-2303-01.xml'))
    # need to update the gains for the mirror group also
    gains_cmd = hebi.GroupCommand(1)
    gains_cmd.read_gains(os.path.join(base_dir, 'gains/mirror_shoulder.xml'))
    mirror_group.send_command_with_acknowledgement(gains_cmd)

    output_arm.cancel_goal()

    gripper_group = lookup.get_group_from_names(['Arm'], ['gripperSpool'])
    while gripper_group is None:
        m.clear_text(blocking=False)
        m.add_text("Looking for gripper module 'Arm/gripperSpool' ...", blocking=False)
        sleep(1)
        gripper_group = lookup.get_group_from_names(['Arm'], ['gripperSpool'])

    gripper = Gripper(gripper_group, -5, 1)
    gripper.load_gains(os.path.join(os.path.dirname(__file__), '../../../kits/arm/gains/gripper_spool_gains.xml'))

    input_arm.update()
    output_arm.update()

    output_joints_home = [0.0, -1.0, 0.0, -0.6, -np.pi / 2, 1.0, 0.0]
    # allowed angular difference (°) per joint before starting align
    allowed_diff = np.array([30.0, 20.0, 30.0, 20.0, 45.0, 45.0, 360.0])

    leader_follower_control = LeaderFollowerControl(input_arm, output_arm, output_joints_home, allowed_diff)
    gripper_control = GripperControl(gripper)

    # Update text/border color on state transitions

    def update_ui(controller: LeaderFollowerControl, new_state: LeaderFollowerControlState):
        if controller.state == new_state:
            return

        if new_state == LeaderFollowerControlState.HOMING:
            m.set_led_color('blue', blocking=False)
            m.clear_text(blocking=False)
            m.add_text("Homing...", blocking=False)
        elif new_state == LeaderFollowerControlState.UNALIGNED:
            m.set_led_color('blue', blocking=False)
        elif new_state == LeaderFollowerControlState.ALIGNED:
            m.set_led_color('green', blocking=False)
            m.clear_text(blocking=False)
            m.add_text("Aligned!", blocking=False)

    leader_follower_control._transition_handlers.append(update_ui)

    last_text_update = 0.0
    while leader_follower_control.running and gripper_control.running:
        t = time()
        try:
            arm_inputs, gripper_inputs = parse_mobile_feedback(m)
            if arm_inputs is None:
                arm_inputs = LeaderFollowerInputs(False)
            leader_follower_control.update(t, arm_inputs)
            gripper_control.update(t, gripper_inputs)
            leader_follower_control.send()
            gripper_control.send()
            if leader_follower_control.state == LeaderFollowerControlState.UNALIGNED:
                if t - last_text_update > 0.1:
                    last_text_update = t
                    m.clear_text(blocking=False)
                    m.add_text(f'Unaligned: {np.around(np.rad2deg(leader_follower_control.angle_diff), decimals=0)}', blocking=False)
        except KeyboardInterrupt:
            leader_follower_control.transition_to(LeaderFollowerControlState.EXIT)
