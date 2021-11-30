#!/usr/bin/env python3

from enum import Enum, auto
from time import sleep
import numpy as np

import hebi

class ContinuousAngleMaps:

    def __init__(self, group):
        self.group = group
        self.group_fbk = hebi.GroupFeedback(group.size)
        self.angle_offsets = np.zeros(group.size)
        self.prev_angles = np.zeros(group.size)

        self.group.feedback_frequency = 200.0

    def update(self):
        self.group.get_next_feedback(reuse_fbk=self.group_fbk)
        diffs = self.group_fbk.position - self.prev_angles
        self.prev_angles = self.group_fbk.position

        for i, diff in enumerate(diffs):
            if diff > np.pi:
                self.angle_offsets[i] -= 2*np.pi 
            elif diff < -np.pi:
                self.angle_offsets[i] += 2*np.pi 

    def print_angles(self):
        print(f'Angles: {np.around(np.rad2deg(self.positions), decimals=0)}')

    @property
    def position(self):
        return self.angle_offsets + self.group_fbk.position


class MimicDemoState(Enum):
    Aligned = auto()
    Unaligned = auto()
    Exit = auto()


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    maps_group = lookup.get_group_from_names(['MAPS'], ['J1-A', 'J2-B', 'J2-A', 'J3-B', 'J3-A', 'J4-B', 'J4-A'])
    if maps_group is None:
        print('MAPS arm not found: Check connection and make sure all modules are blinking green')
        exit(1)

    input_arm = ContinuousAngleMaps(maps_group)

    robot_model = hebi.robot_model.import_from_hrdf('7DoF_arm.hrdf')
    output_arm = hebi.arm.create(
            ['Arm'],
            ['J1_base', 'J2_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2'],
            hrdf_file='7DoF_arm.hrdf',
            lookup=lookup)

    output_arm.cancel_goal()
    output_goal = hebi.arm.Goal(output_arm.size)

    demo_state = MimicDemoState.Unaligned

    input_arm.update()
    output_arm.update()
    angle_remainders = output_arm.last_feedback.position % np.pi
    angle_offsets = output_arm.last_feedback.position - angle_remainders
    print(angle_offsets)

    while demo_state != MimicDemoState.Exit:
        input_arm.update()
        output_arm.update()
        adjusted_output_position = output_arm.last_feedback.position - angle_offsets
        diff = adjusted_output_position - input_arm.position

        if demo_state == MimicDemoState.Aligned:
            output_goal.clear()
            # change this t value to adjust how "snappy" the output arm is to the input arm's position
            output_goal.add_waypoint(t=0.1, position=input_arm.position + angle_offsets)
            output_arm.set_goal(output_goal)

            # Uncomment this section and the output arm can "disconnect"
            # if the angle difference between the two arms gets too large
            # if np.rad2deg(np.max(np.abs(diff))) > 30.0:
            #     demo_state = MimicDemoState.Unaligned
        elif demo_state == MimicDemoState.Unaligned:
            print(f'Diffs: {np.around(np.rad2deg(diff), decimals=0)}')
            if np.rad2deg(np.max(np.abs(diff))) <= 15.0:
                print('Arm Aligned!')
                demo_state = MimicDemoState.Aligned

        output_arm.send()
