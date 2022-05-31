#!/usr/bin/env python3

import os
from time import time, sleep
import numpy as np

import hebi
from hebi.arm import Gripper

from gripper_control import GripperControl, GripperInputs
from kbhit import KBHit

from MAPS_input_device_example import ContinuousAngleMaps, LeaderFollowerControl, LeaderFollowerControlState, LeaderFollowerInputs

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    maps_group = lookup.get_group_from_names(['MAPS'], ['J1-A', 'J2-B', 'J2-A', 'J3-B', 'J3-A', 'J4-B', 'J4-A'])
    if maps_group is None:
        print('MAPS arm not found: Check connection and make sure all modules are blinking green')
        exit(1)

    # need these b/c MAPS joint zeros are in different locations
    angle_offsets = np.array([0.0, np.pi / 2, -np.pi, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
    input_arm = ContinuousAngleMaps(maps_group, angle_offsets)
    output_arm = hebi.arm.create(
        ['Arm'],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2'],
        hrdf_file='hrdf/A-2303-01.hrdf',
        lookup=lookup)

    mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    while mirror_group is None:
        print('Still looking for mirror group...')
        sleep(1)
        mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    # mirror the position/velocity/effort of module 1 ('J2A_shoulder1') to the module
    # in the mirror group ('J2B_shoulder1')
    # Keeps the two modules in the double shoulder bracket in sync
    output_arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    output_arm.load_gains('gains/A-2303-01.xml')
    # need to update the gains for the mirror group also
    gains_cmd = hebi.GroupCommand(1)
    gains_cmd.read_gains('gains/mirror_shoulder.xml')
    mirror_group.send_command_with_acknowledgement(gains_cmd)

    output_arm.cancel_goal()

    gripper_group = lookup.get_group_from_names(['Arm'], ['gripperSpool'])
    while gripper_group is None:
        print("Looking for gripper module 'Arm/gripperSpool' ...")
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

    arm_inputs = LeaderFollowerInputs()
    gripper_strength = 0.0
    gripper_inputs = GripperInputs(gripper_strength)

    print('--- Press Spacebar to Toggle Gripper ---')
    kb = KBHit()
    last_press_time = time()
    pressed = False
    while leader_follower_control.running and gripper_control.running:
        t = time()
        try:
            if kb.kbhit():
                c = kb.getch()
                if ord(c) == 32:  # Spacebar
                    last_press_time = t
                    if not pressed:
                        pressed = True
                        gripper_strength = 1.0 - gripper_strength
                        gripper_inputs = GripperInputs(gripper_strength)
            
            if pressed and t - last_press_time > 0.5:
                pressed = False


            leader_follower_control.update(t, arm_inputs)
            gripper_control.update(t, gripper_inputs)
            leader_follower_control.send()
            gripper_control.send()
        except KeyboardInterrupt:
            leader_follower_control.transition_to(LeaderFollowerControlState.EXIT)
