#!/usr/bin/env python3

import os
from time import time, sleep
import numpy as np
from kits.camera.camera import HebiCamera

import hebi
from hebi.robot_model import endeffector_position_objective
from hebi.robot_model import endeffector_so3_objective
from hebi.robot_model import endeffector_tipaxis_objective

from .MAPS_input_device_example import ContinuousAngleMaps, LeaderFollowerControlState

import typing
if typing.TYPE_CHECKING:
    from typing import Optional, Callable
    import numpy.typing as npt
    from hebi.arm import Arm


class LeaderFollowerInputs:

    def __init__(self, align=False):
        self.align = align


class LeaderFollowerControl:
    def __init__(self, input_arm: ContinuousAngleMaps, output_arm: 'Arm', output_arm_home: 'list[float] | npt.NDArray[np.float64]', allowed_diff: 'list[float] | npt.NDArray[np.float64]'):
        self.state = LeaderFollowerControlState.STARTUP
        self.last_input_time = time()
        self.last_update_time = self.last_input_time
        self.input_arm = input_arm

        self.allowed_diffs = np.array(allowed_diff)
        self.xyz_offset = np.zeros(3)
        self.rot_offset = np.eye(3)

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
            orientation_out = np.zeros((3, 3))
            output_xyz = self.output_arm.FK(self.output_arm.last_feedback.position, orientation_out=orientation_out)
            orientation_in = input_fk[:3, :3]
            input_xyz = input_fk[:3, 3]
            output_tipaxis = orientation_out[:, 2]
            input_tipaxis = input_fk[:3, 2]
            diff_xyz = output_xyz - input_xyz

            Δrot_total = np.arccos(((orientation_in @ orientation_out.T).trace() - 1.0) / 2)
            Δrot_total = np.rad2deg(Δrot_total)

            Δrot_axis = np.arccos((input_fk[:3, 2] @ output_tipaxis))
            Δrot_axis = np.rad2deg(Δrot_axis)

            msg = f'Tip: {np.around(Δrot_axis, decimals=0)}°\nTotal: {np.around(Δrot_total, decimals=0)}°'
            m.clear_text()
            m.add_text(msg)

            if command_input.align and np.abs(Δrot_total) < 5:
                self.xyz_offset = diff_xyz
                #self.rot_offset = orientation_out @ orientation_in.T
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
                Δrot = np.arccos(((input_rot @ self.input_rot_home.T).trace() - 1.0) / 2)
                Δrot = np.deg2rad(Δrot)

                print(f'Pos delta: {np.around(input_xyz - self.input_xyz_home, decimals=2)} | Rot delta: {np.around(Δrot, decimals=2)}')

                ik_angles = self.output_arm.last_feedback.position_command
                if np.any(np.isnan(ik_angles)):
                    ik_angles = self.output_arm.last_feedback.position

                self.output_arm.robot_model.solve_inverse_kinematics(ik_angles,
                                                                     endeffector_position_objective(input_xyz + self.xyz_offset),
                                                                     endeffector_so3_objective(self.rot_offset @ input_rot),
                                                                     #endeffector_tipaxis_objective(input_rot[:, 2]),
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

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = hebi.util.create_mobile_io(lookup, 'Tready')
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = hebi.util.create_mobile_io(lookup, 'Tready')

    m.update()
    m.resetUI()
    for i in range(8):
        m.set_button_label(i + 1, '')
        m.set_axis_label(i + 1, '')

    m.set_button_label(1, 'start')
    m.set_button_label(8, 'quit')

    m.set_axis_label(3, 'flood')
    m.set_axis_label(4, 'spot')
    m.set_axis_label(5, 'zoom')

    m.set_axis_value(3, -1.0)
    m.set_axis_value(4, -1.0)
    m.set_axis_value(5, -1.0)

    zoom_group = lookup.get_group_from_names('C10', ['C10-0003'])
    while zoom_group is None:
        print('Looking for zoom camera...')
        sleep(1)
        zoom_group = lookup.get_group_from_names('C10', ['C10-0003'])

    zoom_camera = HebiCamera(zoom_group)

    maps_group = lookup.get_group_from_names(['MAPS'], ['J1-A', 'J2-B', 'J2-A', 'J3-B', 'J3-A', 'J4-B', 'J4-A'])
    if maps_group is None:
        print('MAPS arm not found: Check connection and make sure all modules are blinking green')
        exit(1)

    # need these b/c MAPS joint zeros are in different locations
    input_arm = ContinuousAngleMaps(maps_group, [0] * 7)

    hrdf_file = 'hrdf/A-2240-06C.hrdf'
    gains_file = 'gains/A-2240-06.xml'

    filename = os.path.abspath(__file__)
    root_dir = filename.split('advanced')[0]

    hrdf_file = os.path.join(root_dir, 'kits/arm', hrdf_file)
    gains_file = os.path.join(root_dir, 'kits/arm', gains_file)

    output_arm = hebi.arm.create(
        ['Arm'],
        ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=hrdf_file,
        lookup=lookup)

    output_arm.load_gains(gains_file)

    output_arm.cancel_goal()

    input_arm.update()
    output_arm.update()

    output_joints_home = [np.pi, 0.55, -2.50, np.pi / 2, np.pi / 2, np.pi / 2]

    leader_follower_control = LeaderFollowerControl(input_arm, output_arm, output_joints_home, allowed_diff=[0.1, 0.1, 0.1])
    #gripper_control = GripperControl(gripper)

    # Because we don't need mobileIO for this demo, just initialize this at the beginning
    arm_inputs = LeaderFollowerInputs()

    roll_cmd = hebi.GroupCommand(1)

    while leader_follower_control.running:
        t = time()
        try:
            if m.update(0.0):
                roll_cmd.io.c.set_float(1, zoom_camera.roll)
                m._group.send_command(roll_cmd)
                zoom_camera.flood_light = (m.get_axis_state(3) + 1.0) / 2
                zoom_camera.spot_light = (m.get_axis_state(4) + 1.0) / 2
                zoom_camera.zoom_level = (m.get_axis_state(5) + 1.0) / 2

                if m.get_button_state(1):
                    arm_inputs.align = True

                if m.get_button_state(8):
                    leader_follower_control.transition_to(LeaderFollowerControlState.EXIT)

            leader_follower_control.update(t, arm_inputs)
            zoom_camera.update()

            leader_follower_control.send()
            zoom_camera.send()
        except KeyboardInterrupt:
            leader_follower_control.transition_to(LeaderFollowerControlState.EXIT)
