#!/usr/bin/env python3

import os
from time import time, sleep
import numpy as np

import hebi

import typing
if typing.TYPE_CHECKING:
    import numpy.typing as npt
    from hebi.arm import Arm

from .MAPS_input_device_example import ContinuousAngleMaps


def angle_between(v1: 'npt.NDArray[np.float64]', v2: 'npt.NDArray[np.float64]') -> float:
    mag_v1 = np.sqrt(v1.dot(v1))
    mag_v2 = np.sqrt(v2.dot(v2))
    return np.arccos(v1.dot(v2) / (mag_v1 * mag_v2))


def get_frame_tip_axis(arm: 'Arm', frame_num):
    frames = arm.robot_model.get_forward_kinematics('output', arm.last_feedback.position)
    upper_arm_frame = frames[frame_num]  # TODO: Can this be figured out without hardcoding?
    return upper_arm_frame[:3, 2]


def get_frame_tip_axes(arm: 'Arm'):
    frames = arm.robot_model.get_forward_kinematics('output', arm.last_feedback.position)
    return [f[:3, 2] for f in frames]


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

    hrdf_file = 'hrdf/7-DoF-Maggie.hrdf'
    root_dir = os.path.dirname(__file__)
    hrdf_file = os.path.join(root_dir, hrdf_file)

    output_arm = hebi.arm.create(
        ['Maggie-Arm'],
        ['J1_base', 'J2_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2'],
        hrdf_file=hrdf_file,
        lookup=lookup)

    output_arm.cancel_goal()
    g = hebi.arm.Goal(output_arm.size)
    g.add_waypoint(t=10, position=[0.8, 3.3, -3.6, -0.7, -1.7, 0.78, -0.12])

    input_arm.update()
    output_arm.update()
    output_arm.set_goal(g)

    while True:
        try:
            input_arm.update()
            output_arm.update()
            print('----------------------------------')
            in_elbow_tip_axis = input_arm.get_upper_arm_tip_axis()
            angle = angle_between(in_elbow_tip_axis, get_frame_tip_axis(output_arm, 5))
            print(f'Frame #5: {np.around(np.rad2deg(angle), decimals=1)}')

            print('----------------------------------')
            for i, ta in enumerate(get_frame_tip_axes(output_arm)):
                angle = angle_between(in_elbow_tip_axis, ta)
                print(f'Frame #{i}: {np.around(np.rad2deg(angle), decimals=1)}')
            output_arm.send()
        except KeyboardInterrupt:
            break
