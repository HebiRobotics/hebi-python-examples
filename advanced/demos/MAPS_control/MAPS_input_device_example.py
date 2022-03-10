#!/usr/bin/env python3

import os
from enum import Enum, auto
from time import time, sleep
import numpy as np

import hebi
from hebi.robot_model import endeffector_position_objective
from hebi.robot_model import endeffector_so3_objective
from hebi.robot_model import custom_objective

import typing
if typing.TYPE_CHECKING:
    from typing import Optional
    import numpy.typing as npt
    from hebi.arm import Arm
    from hebi._internal.group import Group


class ContinuousAngleMaps:

    def __init__(self, group: 'Group', offsets):
        self.group = group
        self.group_fbk = hebi.GroupFeedback(group.size)
        self.angle_offsets: 'npt.NDArray[np.float64]' = np.array(offsets)
        self.prev_angles: 'npt.NDArray[np.float64]' = np.zeros(group.size, dtype=np.float64)

        self.group.feedback_frequency = 200.0
        self.input_model = hebi.robot_model.import_from_hrdf('hrdf/mapsArm_7DoF_standard.hrdf')

    def update(self):
        self.group.get_next_feedback(reuse_fbk=self.group_fbk)
        diffs = self.group_fbk.position - self.prev_angles
        self.prev_angles = self.group_fbk.position

        for i, diff in enumerate(diffs):
            if diff > np.pi:
                self.angle_offsets[i] -= 2*np.pi
            elif diff < -np.pi:
                self.angle_offsets[i] += 2*np.pi 

    def rebalance(self):
        '''Adjusts angles so they lie in [-π, π]'''
        for i in range(len(self.angle_offsets)):
            while self.angle_offsets[i] > np.pi:
                self.angle_offsets[i] -= np.pi
            while self.angle_offsets[i] < -np.pi:
                self.angle_offsets[i] += np.pi

    def get_fk(self) -> 'npt.NDArray[np.float64]':
        return np.copy(self.input_model.get_end_effector(self.group_fbk.position))

    @property
    def position(self):
        return self.angle_offsets + self.group_fbk.position


class MimicDemoState(Enum):
    Startup = auto()
    Homing = auto()
    Unaligned = auto()
    Aligning = auto()
    Aligned = auto()
    Exit = auto()


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    maps_group = lookup.get_group_from_names(['MAPS'], ['J1-A', 'J2-B', 'J2-A', 'J3-B', 'J3-A', 'J4-B', 'J4-A'])
    if maps_group is None:
        print('MAPS arm not found: Check connection and make sure all modules are blinking green')
        exit(1)

    # need these b/c MAPS joint zeros are in different locations
    angle_offsets = np.array([0.0, np.pi/2, -np.pi, -np.pi/2, -np.pi/2, -np.pi/2, 0.0])
    input_arm = ContinuousAngleMaps(maps_group, angle_offsets)

    # objective function for use w/ IK later
    def arm_MAPS_mse(positions, errors, user_data):
        errors[0] = np.sum((input_arm.position - positions) ** 2)

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
    output_goal = hebi.arm.Goal(output_arm.size)

    output_xyz_home = np.zeros(3, dtype=np.float64)
    output_rot_home = np.zeros((3,3), dtype=np.float64)
    output_joints_home = [0.0, -1.0, 0.0, -0.6, -np.pi/2, 1.0, 0.0]
    output_arm.FK(output_joints_home,
                  xyz_out=output_xyz_home,
                  orientation_out=output_rot_home)

    print(f'Output Home XYZ: {output_xyz_home}')
    print(f'Output Home ROT:\n{output_rot_home}')

    demo_state = MimicDemoState.Startup

    input_arm.update()
    output_arm.update()

    # used later for storing IK results
    target_joints = np.empty(7, dtype=np.float64)

    # allowed angular difference (°) per joint before starting align
    allowed_diff = np.array([30.0, 20.0, 30.0, 20.0, 45.0, 45.0, 360.0])

    while demo_state != MimicDemoState.Exit:
        try:
            input_arm.update()
            output_arm.update()

            if demo_state == MimicDemoState.Startup:
                # Go to arm home pose
                output_goal.clear()
                output_goal.add_waypoint(t=3.0, position=output_joints_home)
                output_arm.set_goal(output_goal)
                demo_state = MimicDemoState.Homing

            elif demo_state == MimicDemoState.Homing:
                if output_arm.at_goal:
                    demo_state = MimicDemoState.Unaligned

            elif demo_state == MimicDemoState.Unaligned:
                # Wait until all the joints of the arm and MAPS are
                # within allowed_diff of each other, then align

                # Because the arm is commanded to its home position,
                # MAPS angles need to be kept within [-π, π]
                input_arm.rebalance()
                diff = output_arm.last_feedback.position - input_arm.position

                print(f'Diffs: {np.around(np.rad2deg(diff), decimals=0)}')
                if np.all(np.rad2deg(np.abs(diff)) <= allowed_diff):
                    print("Aligning w/ MAPS config")
                    input_fk = input_arm.get_fk()
                    input_xyz_home = input_fk[:3, 3]
                    input_rot_home = input_fk[:3, :3]
                    print(f'MAPS xyz home: {input_xyz_home}')
                    print(f'MAPS rot home:\n{input_rot_home}')

                    output_goal.clear()
                    output_goal.add_waypoint(t=2.0, position=input_arm.position)
                    last_angles = input_arm.position
                    output_arm.set_goal(output_goal)
                    demo_state = MimicDemoState.Aligning

            elif demo_state == MimicDemoState.Aligning:
                # If we're done aligning the arm to the MAPS arm's geometry,
                # Update the arm's home pose and start actively tracking
                if output_arm.at_goal:
                    output_joints_home = output_arm.last_feedback.position_command
                    output_arm.FK(output_joints_home,
                                  xyz_out=output_xyz_home,
                                  orientation_out=output_rot_home)
                    demo_state = MimicDemoState.Aligned

            elif demo_state == MimicDemoState.Aligned:
                # if the MAPS angles have changed enough from the previous value
                if np.any(np.abs(input_arm.position - last_angles) > 0.01):
                    last_angles = input_arm.position

                    input_fk = input_arm.get_fk()
                    input_xyz = input_fk[:3, 3]
                    input_rot = input_fk[:3, :3]

                    xyz_target = output_xyz_home + (input_xyz - input_xyz_home)
                    rot_target = np.matmul(np.matmul(input_rot, input_rot_home.T), output_rot_home)

                    # Calculate new arm joint angles
                    # seed IK with the last commanded arm position
                    # Three objectives:
                    # Cartesian end effector position
                    # End effector orientation
                    # Mean Squared Error between arm joint angles and MAPS input arm angles
                    # result written into target_joints
                    output_arm.robot_model.solve_inverse_kinematics(output_arm.last_feedback.position_command,
                                                                    endeffector_position_objective(xyz_target),
                                                                    endeffector_so3_objective(rot_target),
                                                                    custom_objective(1, arm_MAPS_mse, weight=0.1),
                                                                    output=target_joints)

                    output_goal.clear()
                    # change this t value to adjust how "snappy" the output arm is to the input arm's position
                    output_goal.add_waypoint(t=0.3, position=target_joints)
                    output_arm.set_goal(output_goal)

            output_arm.send()
        except KeyboardInterrupt:
            demo_state = MimicDemoState.Exit

    exit()
