#!/usr/bin/env python3

import os
from time import time, sleep
import numpy as np

from .gripper_control import GripperControl, GripperInputs
from .kbhit import KBHit

import hebi
from hebi.arm import Gripper
from hebi.robot_model import endeffector_position_objective
from hebi.robot_model import endeffector_so3_objective

from .MAPS_input_device_example import ContinuousAngleMaps, LeaderFollowerControlState, LeaderFollowerInputs

import typing
if typing.TYPE_CHECKING:
    from typing import Optional, Callable
    import numpy.typing as npt
    from hebi.arm import Arm


class LeaderFollowerControl:
    def __init__(self, input_arm: ContinuousAngleMaps, output_arm: 'Arm', output_arm_home: 'list[float] | npt.NDArray[np.float64]', allowed_diff: 'list[float] | npt.NDArray[np.float64]'):
        self.state = LeaderFollowerControlState.STARTUP
        self.last_input_time = time()
        self.last_update_time = self.last_input_time
        self.input_arm = input_arm

        self.allowed_diffs = np.array(allowed_diff)

        self.output_arm = output_arm
        self.output_goal = hebi.arm.Goal(output_arm.size)
        self.output_arm_home = output_arm_home
        # These are calculated later
        self.output_xyz_home: 'npt.NDArray[np.float64]' = np.empty(3, dtype=np.float64)
        self.output_rot_home: 'npt.NDArray[np.float64]' = np.empty((3, 3), dtype=np.float64)

        # used later for storing IK results
        self.target_joints: 'npt.NDArray[np.float64]' = np.empty(output_arm.size, dtype=np.float64)

        self._transition_handlers: 'list[Callable[[LeaderFollowerControl, LeaderFollowerControlState], None]]' = []

    @property
    def running(self):
        return self.state is not self.state.EXIT

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
            # Wait until all the MAPS end effector is near
            # arm output position, then align

            input_fk = self.input_arm.get_fk()
            tip_axis = np.zeros(3)
            output_xyz = self.output_arm.FK(self.output_arm.last_feedback.position, tip_axis_out=tip_axis)
            diff_xyz = output_xyz - input_fk[:3, 3]
            diff_axis = tip_axis - input_fk[:3, 2]

            print(f'Diff xyz: {np.around(diff_xyz, decimals=3)} | Tip: {np.around(diff_axis, decimals=3)}')
            if np.all(np.abs(diff_xyz) <= self.allowed_diffs):
                self.transition_to(self.state.ALIGNING)

        elif self.state is self.state.ALIGNING:
            # If we're done aligning the arm to the MAPS arm's geometry,
            # Update the arm's home pose and start actively tracking
            self.last_input_position = self.input_arm.position
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
                #rot_target = np.matmul(np.matmul(input_rot, self.input_rot_home.T), self.output_rot_home)
                rot_target = input_rot @ self.input_rot_home.T @ self.output_rot_home

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

                ik_angles[2] = abs(ik_angles[2])

                self.output_arm.robot_model.solve_inverse_kinematics(ik_angles,
                                                                     endeffector_position_objective(xyz_target),
                                                                     endeffector_so3_objective(rot_target),
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
            self.output_goal.add_waypoint(t=10, position=self.output_arm_home)
            print(self.output_arm_home)
            self.output_arm.set_goal(self.output_goal)

        elif state is self.state.ALIGNING:
            print("TRANSITIONING TO ALIGNING")
            input_fk = self.input_arm.get_fk()
            self.input_xyz_home = input_fk[:3, 3]
            self.input_rot_home = input_fk[:3, :3]
            print(f'MAPS xyz home: {self.input_xyz_home}')
            print(f'MAPS rot home:\n{self.input_rot_home}')

        elif state is self.state.ALIGNED:
            print("TRANSITIONING TO ALIGNED")
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
    input_arm = ContinuousAngleMaps(maps_group)

    hrdf_file = 'hrdf/A-2085-06G.hrdf'
    gains_file = 'gains/A-2085-06.xml'

    filename = os.path.abspath(__file__)
    root_dir = filename.split('advanced')[0]

    hrdf_file = os.path.join(root_dir, 'kits/arm', hrdf_file)
    gains_file = os.path.join(root_dir, 'kits/arm', gains_file)

    output_arm = hebi.arm.create(
        ['Rosie'],
        ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=hrdf_file,
        lookup=lookup)

    output_arm.load_gains(gains_file)

    output_arm.cancel_goal()

    gripper_group = lookup.get_group_from_names(['Rosie'], ['gripperSpool'])
    while gripper_group is None:
        print("Looking for gripper module 'Rosie/gripperSpool' ...")
        sleep(1)
        gripper_group = lookup.get_group_from_names(['Rosie'], ['gripperSpool'])

    gripper = Gripper(gripper_group, -5, 1)
    gripper.load_gains(os.path.join(os.path.dirname(__file__), '../../../kits/arm/gains/gripper_spool_gains.xml'))

    input_arm.update()
    output_arm.update()

    output_joints_home = [0.0, 2.0, 2.0, 1.57, -1.57, 0.0]
    # allowed angular difference (°) per joint before starting align

    leader_follower_control = LeaderFollowerControl(input_arm, output_arm, output_joints_home, allowed_diff=[0.05, 0.05, 0.05])
    gripper_control = GripperControl(gripper)

    # Because we don't need mobileIO for this demo, just initialize this at the beginning
    arm_inputs = LeaderFollowerInputs()
    gripper_strength = 0.0
    gripper_inputs = GripperInputs(gripper_strength)

    print('--- Press Spacebar to Toggle Gripper ---')
    kb = KBHit()
    while leader_follower_control.running and gripper_control.running:
        t = time()
        try:
            if kb.kbhit():
                c = kb.getch()
                if ord(c) == 32:  # Spacebar
                    gripper_strength = 1.0 - gripper_strength
                    gripper_inputs = GripperInputs(gripper_strength)

            leader_follower_control.update(t, arm_inputs)
            gripper_control.update(t, gripper_inputs)
            leader_follower_control.send()
            gripper_control.send()
        except KeyboardInterrupt:
            leader_follower_control.transition_to(LeaderFollowerControlState.EXIT)
