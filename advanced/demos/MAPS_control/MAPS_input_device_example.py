#!/usr/bin/env python3

import os
from enum import Enum, auto
from time import time, sleep
import numpy as np

import hebi
from hebi.robot_model import endeffector_position_objective, PositionObjective
from hebi.robot_model import endeffector_so3_objective, SO3Objective
from hebi.robot_model import TipAxisObjective
from hebi.robot_model import custom_objective

import typing
if typing.TYPE_CHECKING:
    from typing import Optional, Callable
    import numpy.typing as npt
    from hebi.arm import Arm
    from hebi._internal.group import Group


def angle_between(v1: 'npt.NDArray[np.float64]', v2: 'npt.NDArray[np.float64]') -> float:
    mag_v1 = np.sqrt(v1.dot(v1))
    mag_v2 = np.sqrt(v2.dot(v2))
    return np.arccos(v1.dot(v2) / (mag_v1 * mag_v2))


class ContinuousAngleMaps:

    def __init__(self, group: 'Group', offsets):
        self.group = group
        self.group_fbk = hebi.GroupFeedback(group.size)
        self.angle_offsets: 'npt.NDArray[np.float64]' = np.array(offsets)
        self.prev_angles: 'npt.NDArray[np.float64]' = np.zeros(group.size, dtype=np.float64)

        self.group.feedback_frequency = 200.0
        base_dir, _ = os.path.split(__file__)
        hrdf_file = os.path.join(base_dir, 'hrdf/mapsArm_7DoF_snakeControl.hrdf')
        self.input_model = hebi.robot_model.import_from_hrdf(hrdf_file)

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

    def get_upper_arm_orientation(self) -> 'npt.NDArray[np.float64]':
        frames = self.input_model.get_forward_kinematics('output', self.group_fbk.position)
        upper_arm_frame = frames[9]  # TODO: Can this be figured out without hardcoding?
        return upper_arm_frame[:3, :3]

    def get_upper_arm_tip_axis(self) -> 'npt.NDArray[np.float64]':
        frames = self.input_model.get_forward_kinematics('output', self.group_fbk.position)
        upper_arm_frame = frames[9]  # TODO: Can this be figured out without hardcoding?
        return upper_arm_frame[:3, 2]

    def get_lower_arm_orientation(self) -> 'npt.NDArray[np.float64]':
        frames = self.input_model.get_forward_kinematics('output', self.group_fbk.position)
        lower_arm_frame = frames[15]  # TODO: Can this be figured out without hardcoding?
        return lower_arm_frame[:3, :3]

    def get_lower_arm_tip_axis(self) -> 'npt.NDArray[np.float64]':
        frames = self.input_model.get_forward_kinematics('output', self.group_fbk.position)
        lower_arm_frame = frames[15]  # TODO: Can this be figured out without hardcoding?
        return lower_arm_frame[:3, 2]

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

    def __init__(self, reset=False):
        self.reset = reset


class LeaderFollowerControl:
    def __init__(self, input_arm: ContinuousAngleMaps, output_arm: 'Arm', output_arm_home: 'list[float] | npt.NDArray[np.float64]', alignment_diffs):
        self.state = LeaderFollowerControlState.STARTUP
        self.last_input_time = time()
        self.last_update_time = self.last_input_time
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
        self._transition_handlers: 'list[Callable[[LeaderFollowerControl, LeaderFollowerControlState], None]]' = []

    @property
    def running(self):
        return self.state is not self.state.EXIT

    # objective function for use w/ IK later
    def arm_MAPS_mse(self, positions: 'npt.NDArray[np.float64]', errors: 'npt.NDArray[np.float64]', user_data: None):
        errors[0] = np.sum((self.input_arm.position - positions) ** 2)

    def send(self):
        self.output_arm.send()

    @property
    def angle_diff(self):
        output_pos = self.output_arm.last_feedback.position
        self.input_arm.rebalance(output_pos)
        return output_pos - self.input_arm.position

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

            if command_input.reset:
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

            input_fk = self.input_arm.get_fk()
            out_tip_axis = np.zeros(3)
            angles = self.output_arm.last_feedback.position
            output_xyz = self.output_arm.FK(angles, tip_axis_out=out_tip_axis)

            diff_xyz = output_xyz - input_fk[:3, 3]
            diff_tip = np.rad2deg(angle_between(input_fk[:3, 2], out_tip_axis))

            upper_arm_tip_axis = self.input_arm.get_upper_arm_tip_axis()
            frames = self.output_arm.robot_model.get_forward_kinematics('output', angles)
            upper_arm_frame = frames[5]  # TODO: Can this be figured out without hardcoding?
            out_elbow_tip_axis = upper_arm_frame[:3, 2]
            diff_axis = np.rad2deg(angle_between(upper_arm_tip_axis, out_elbow_tip_axis))

            print((f'Diff xyz: {np.around(diff_xyz, decimals=3)} |'
                   f' Arm: {np.around(diff_axis, decimals=1)} |'
                   f' Tip: {np.around(diff_tip, decimals=1)}'))
            if np.all(np.abs(diff_xyz) <= self.allowed_diffs) \
               and abs(diff_axis) < 15.0 \
               and abs(diff_tip) < 15.0:
                self.transition_to(self.state.ALIGNING)

        elif self.state is self.state.ALIGNING:
            # If we're done aligning the arm to the MAPS arm's geometry,
            # Update the arm's home pose and start actively tracking
            if self.output_arm.at_goal:
                self.transition_to(self.state.ALIGNED)

        elif self.state is self.state.ALIGNED:
            # if the MAPS angles have changed enough from the previous value
            dt = t_now - self.last_update_time
            position_changed = np.any(np.abs(self.input_arm.position - self.last_input_position) > 0.01)
            if dt > 0.05 and position_changed:
                self.last_input_position = self.input_arm.position
                self.last_update_time = t_now

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

                ik_angles = self.output_arm.last_feedback.position_command
                if np.any(np.isnan(ik_angles)):
                    ik_angles = self.output_arm.last_feedback.position

                upper_arm_tip_axis = self.input_arm.get_upper_arm_tip_axis()
                # frames = self.output_arm.robot_model.get_forward_kinematics('output', ik_angles)
                # upper_arm_frame = frames[5]  # TODO: Can this be figured out without hardcoding?
                # out_elbow_tip_axis = upper_arm_frame[:3, 2]
                lower_arm_orientation = self.input_arm.get_lower_arm_orientation()
                lower_arm_tip_axis = lower_arm_orientation[:, 2]
                # pull out current shoulder tip-axis for debug
                frames = self.output_arm.robot_model.get_forward_kinematics('output', ik_angles)
                upper_arm_frame = frames[9]  # TODO: Can this be figured out without hardcoding?
                out_lower_arm_orientation = upper_arm_frame[:3, :3]

                # print(f' In: {np.around(in_elbow_tip_axis, decimals=2)}\n'
                #       f'Out: {np.around(out_elbow_tip_axis, decimals=2)}')

                upper_arm_tip_axis_objective = TipAxisObjective('output',
                                                                idx=5,
                                                                axis=upper_arm_tip_axis,
                                                                weight=0.0)

                lower_arm_tip_axis_objective = TipAxisObjective('output',
                                                                idx=9,
                                                                axis=lower_arm_tip_axis,
                                                                weight=0.0)

                self.output_arm.robot_model.solve_inverse_kinematics(ik_angles,
                                                                     endeffector_position_objective(xyz_target),
                                                                     endeffector_so3_objective(rot_target),
                                                                     upper_arm_tip_axis_objective,
                                                                     lower_arm_tip_axis_objective,
                                                                     #custom_objective(1, self.arm_MAPS_mse, weight=0.1),
                                                                     output=self.target_joints)

                self.output_goal.clear()
                # change this t value to adjust how "snappy" the output arm is to the input arm's position
                self.output_goal.add_waypoint(t=0.3,
                                              position=self.target_joints,
                                              velocity=[np.nan] * self.output_arm.size)
                self.output_arm.set_goal(self.output_goal)

    def transition_to(self, state: LeaderFollowerControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print("TRANSITIONING TO HOMING")
            # Go to arm home pose
            self.output_goal.clear()
            self.output_goal.add_waypoint(t=0.5, position=np.full(self.output_arm.size, np.nan), velocity=[0] * self.output_arm.size)
            self.output_goal.add_waypoint(t=9.5, position=self.output_arm_home)
            self.output_arm.set_goal(self.output_goal)

        elif state is self.state.ALIGNING:
            print("TRANSITIONING TO ALIGNING")
            input_fk = self.input_arm.get_fk()
            self.input_xyz_home = input_fk[:3, 3]
            self.input_rot_home = input_fk[:3, :3]
            print(f'MAPS xyz home: {self.input_xyz_home}')
            print(f'MAPS rot home:\n{self.input_rot_home}')

            # self.output_goal.clear()
            # self.output_goal.add_waypoint(t=2.0, position=self.input_arm.position)
            # self.last_input_position = self.input_arm.position
            # self.output_arm.set_goal(self.output_goal)

        elif state is self.state.ALIGNED:
            print("TRANSITIONING TO ALIGNED")
            self.last_input_position = self.input_arm.group_fbk.position
            curr_pos = self.output_arm.last_feedback.position_command
            if np.any(np.isnan(curr_pos)):
                curr_pos = self.output_arm.last_feedback.position

            # Why did I do this?
            #self.output_arm_home = curr_pos
            self.output_arm.FK(curr_pos,
                               xyz_out=self.output_xyz_home,
                               orientation_out=self.output_rot_home)

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        for handler in self._transition_handlers:
            handler(self, state)
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

    hrdf_file = 'hrdf/7-DoF-Maggie.hrdf'
    root_dir = os.path.dirname(__file__)
    hrdf_file = os.path.join(root_dir, hrdf_file)

    output_arm = hebi.arm.create(
        ['Maggie-Arm'],
        ['J1_base', 'J2_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2'],
        hrdf_file=hrdf_file,
        lookup=lookup)

    # mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    # while mirror_group is None:
    #     print('Still looking for mirror group...')
    #     sleep(1)
    #     mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    # mirror the position/velocity/effort of module 1 ('J2A_shoulder1') to the module
    # in the mirror group ('J2B_shoulder1')
    # Keeps the two modules in the double shoulder bracket in sync
    # output_arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    # output_arm.load_gains('gains/7-DoF-Maggie.xml')
    # need to update the gains for the mirror group also
    # gains_cmd = hebi.GroupCommand(1)
    # gains_cmd.read_gains('gains/mirror_shoulder.xml')
    # mirror_group.send_command_with_acknowledgement(gains_cmd)

    output_arm.cancel_goal()

    input_arm.update()
    output_arm.update()

    #output_joints_home = [0.8, 3.3, -3.6, -0.7, -1.7, -5.5, -6.4]
    output_joints_home = [0.3, 4.5, -3.0, -1.0, 1.7, -0.6, 0.0]
    # allowed cartesian difference (m) at the end effector before aligning
    allowed_diff = np.array([0.05, 0.05, 0.05])

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
