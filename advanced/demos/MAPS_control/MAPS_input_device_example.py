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
                self.angle_offsets[i] -= 2 * np.pi
            elif diff < -np.pi:
                self.angle_offsets[i] += 2 * np.pi

    def rebalance(self, home_pose: 'npt.NDArray[np.float64]'):
        """Adjusts angles so they lie within one rotation of the provided
        pose."""
        for i in range(len(self.angle_offsets)):
            while self.angle_offsets[i] - home_pose[i] > 2 * np.pi:
                self.angle_offsets[i] -= 2 * np.pi
            while self.angle_offsets[i] - home_pose[i] < -2 * np.pi:
                self.angle_offsets[i] += 2 * np.pi

    def get_fk(self) -> 'npt.NDArray[np.float64]':
        return np.copy(self.input_model.get_end_effector(self.group_fbk.position))

    @property
    def position(self):
        return self.angle_offsets + self.group_fbk.position


class LeaderFollowerControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    UNALIGNED = auto()
    ALIGNING = auto()
    ALIGNED = auto()
    DISCONNECTED = auto()
    MSTOPPED = auto()
    EXIT = auto()


class LeaderFollowerInputs:
    pass


class LeaderFollowerControl:
    def __init__(self, input_arm: ContinuousAngleMaps, output_arm: 'Arm', output_arm_home: 'list[float] | npt.NDArray[np.float64]', alignment_diffs):
        self.state = LeaderFollowerControlState.STARTUP
        self.last_input_time = time()
        self.input_arm = input_arm

        self.output_arm = output_arm
        self.output_goal = hebi.arm.Goal(output_arm.size)
        self.output_arm_home = output_arm_home
        # These are calculated later
        self.output_xyz_home: 'npt.NDArray[np.float64]' = np.empty(3, dtype=np.float64)
        self.output_rot_home: 'npt.NDArray[np.float64]' = np.empty((3, 3), dtype=np.float64)

        # used later for storing IK results
        self.target_joints: 'npt.NDArray[np.float64]' = np.empty(7, dtype=np.float64)

        self.allowed_diffs: 'npt.NDArray[np.float64]' = np.array(alignment_diffs, dtype=np.float64)

    @property
    def running(self):
        return self.state is not self.state.EXIT

    # objective function for use w/ IK later
    def arm_MAPS_mse(self, positions: 'npt.NDArray[np.float64]', errors: 'npt.NDArray[np.float64]', user_data: None):
        errors[0] = np.sum((self.input_arm.position - positions) ** 2)

    def send(self):
        self.output_arm.send()

    def update(self, t_now: float, command_input: 'Optional[LeaderFollowerInputs]'):
        self.input_arm.update()
        self.output_arm.update()

        if self.state is self.state.EXIT:
            return

        if not np.all(self.output_arm.last_feedback.mstop_state):
            self.transition_to(self.state.MSTOPPED)

        if not command_input:
            if t_now - self.last_input_time > 1.0 and self.state is not self.state.DISCONNECTED:
                print("mobileIO timeout, disabling motion")
                self.transition_to(self.state.DISCONNECTED)
        else:
            self.last_input_time = t_now

            if self.state is self.state.DISCONNECTED:
                self.last_input_time = t_now
                self.transition_to(self.state.HOMING)

        if self.state is self.state.MSTOPPED:
            if np.all(self.output_arm.last_feedback.mstop_state):
                self.transition_to(self.state.UNALIGNED)

        if self.state is self.state.STARTUP:
            self.transition_to(self.state.HOMING)

        elif self.state is self.state.HOMING:
            if self.output_arm.at_goal:
                self.transition_to(self.state.UNALIGNED)

        elif self.state is self.state.UNALIGNED:
            # Wait until all the joints of the arm and MAPS are
            # within allowed_diff of each other, then align

            # Because the arm is commanded to its home position,
            # MAPS angles need to be kept within [-π, π]
            output_pos = self.output_arm.last_feedback.position
            self.input_arm.rebalance(output_pos)
            diff = output_pos - self.input_arm.position

            print(f'Diffs: {np.around(np.rad2deg(diff), decimals=0)}')
            if np.all(np.rad2deg(np.abs(diff)) <= self.allowed_diffs):
                self.transition_to(self.state.ALIGNING)

        elif self.state is self.state.ALIGNING:
            # If we're done aligning the arm to the MAPS arm's geometry,
            # Update the arm's home pose and start actively tracking
            if self.output_arm.at_goal:
                self.transition_to(self.state.ALIGNED)

        elif self.state is self.state.ALIGNED:
            # if the MAPS angles have changed enough from the previous value
            if np.any(np.abs(self.input_arm.position - self.last_input_position) > 0.01):
                self.last_input_position = self.input_arm.position

                input_fk = self.input_arm.get_fk()
                input_xyz = input_fk[:3, 3]
                input_rot = input_fk[:3, :3]

                xyz_target = self.output_xyz_home + (input_xyz - self.input_xyz_home)
                rot_target = np.matmul(np.matmul(input_rot, self.input_rot_home.T), self.output_rot_home)

                # Calculate new arm joint angles
                # seed IK with the last commanded arm position
                # Three objectives:
                # Cartesian end effector position
                # End effector orientation
                # Mean Squared Error between arm joint angles and MAPS input arm angles
                # result written into target_joints

                self.output_arm.robot_model.solve_inverse_kinematics(self.output_arm.last_feedback.position_command,
                                                                     endeffector_position_objective(xyz_target),
                                                                     endeffector_so3_objective(rot_target),
                                                                     custom_objective(1, self.arm_MAPS_mse, weight=0.1),
                                                                     output=self.target_joints)

                self.output_goal.clear()
                # change this t value to adjust how "snappy" the output arm is to the input arm's position
                self.output_goal.add_waypoint(t=0.3, position=self.target_joints)
                self.output_arm.set_goal(self.output_goal)

    def transition_to(self, state: LeaderFollowerControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print("TRANSITIONING TO HOMING")
            # Go to arm home pose
            self.output_goal.clear()
            self.output_goal.add_waypoint(t=10.0, position=self.output_arm_home)
            self.output_arm.set_goal(self.output_goal)

        elif state is self.state.ALIGNING:
            print("TRANSITIONING TO ALIGNING")
            input_fk = self.input_arm.get_fk()
            self.input_xyz_home = input_fk[:3, 3]
            self.input_rot_home = input_fk[:3, :3]
            print(f'MAPS xyz home: {self.input_xyz_home}')
            print(f'MAPS rot home:\n{self.input_rot_home}')

            self.output_goal.clear()
            self.output_goal.add_waypoint(t=2.0, position=self.input_arm.position)
            self.last_input_position = self.input_arm.position
            self.output_arm.set_goal(self.output_goal)

        elif state is self.state.ALIGNED:
            print("TRANSITIONING TO ALIGNED")
            self.output_arm_home = self.output_arm.last_feedback.position_command
            self.output_arm.FK(self.output_arm_home,
                               xyz_out=self.output_xyz_home,
                               orientation_out=self.output_rot_home)

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        self.state = state


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

    input_arm.update()
    output_arm.update()

    output_joints_home = [0.0, -1.0, 0.0, -0.6, -np.pi / 2, 1.0, 0.0]
    # allowed angular difference (°) per joint before starting align
    allowed_diff = np.array([30.0, 20.0, 30.0, 20.0, 45.0, 45.0, 360.0])

    leader_follower_control = LeaderFollowerControl(input_arm, output_arm, output_joints_home, allowed_diff)

    # Because we don't need mobileIO for this demo, just initialize this at the beginning
    arm_inputs = LeaderFollowerInputs()
    while leader_follower_control.running:
        t = time()
        try:
            leader_follower_control.update(t, arm_inputs)
            leader_follower_control.send()
        except KeyboardInterrupt:
            leader_follower_control.transition_to(LeaderFollowerControlState.EXIT)
