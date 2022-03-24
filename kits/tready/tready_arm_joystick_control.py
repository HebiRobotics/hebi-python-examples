import sys
import os
from collections import namedtuple
from time import time, sleep
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

from tready import TreadedBase, TreadyControl, TreadyControlState, ChassisVelocity, TreadyInputs
from tready_utils import set_mobile_io_instructions, setup_base, setup_arm_6dof

import typing
if typing.TYPE_CHECKING:
    from typing import Optional
    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO

def set_mobile_io_mode_text(mobile_io: 'MobileIO', mode_button: int):
    instructions = ('Robot Ready to Control\n'
                    'B1: Reset\n'
                    'B2: {}\n'
                    'B3: Toggle Gripper\n'
                    'B4: Arm Home\n'
                    'B5/7: Arm Up/Down\n'
                    'B6: Joined Flipper\n'
                    'B8 - Quit')

    if mobile_io.get_button_state(mode_button):
        set_mobile_io_instructions(mobile_io, instructions.format('Arm Rot. Mode'), color='yellow')
    else:
        set_mobile_io_instructions(mobile_io, instructions.format('Base Drive Mode'), color='green')


ArmInputs = namedtuple('ArmInputs', ['delta_xyz', 'delta_rot_xyz', 'locked', 'gripper_closed'])
DemoInputs = namedtuple('DemoInputs', ['should_exit', 'should_reset', 'chassis', 'arm'])


class TreadyArmJoystickControl(TreadyControl):

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, mobile_io, base: TreadedBase, arm: hebi.arm.Arm):
        super(TreadyArmJoystickControl, self).__init__(mobile_io, base)

        self.arm = arm

        # 5 DoF home
        # self.arm_xyz_home = [0.55, -0.1, 0.0]
        # 6 DoF home
        self.arm_xyz_home = [0.4, 0.0, 0.0]
        #self.arm_rot_home = R.from_euler('z', np.pi / 2) * R.from_euler('x', np.pi)
        self.arm_rot_home = R.from_euler('y', np.pi)
        self.arm_rot_home = self.arm_rot_home.as_matrix()

        # 5 DoF seed
        # self.arm_seed_ik = np.array([0.01, 0.6, 2, 2.5, 3.14])
        # 6 DoF seed
        self.arm_seed_ik = np.array([0, 1.0, 2, 3, -1.5, 0])
        self.arm_home = self.arm.ik_target_xyz_so3(
            self.arm_seed_ik,
            self.arm_xyz_home,
            self.arm_rot_home)

    def compute_arm_goal(self, arm_inputs: ArmInputs):
        arm = self.arm
        arm_goal = hebi.arm.Goal(arm.size)
        if arm_inputs.locked:
            arm_goal.add_waypoint(t=3.0, position=self.arm_home)
            return arm_goal
        else:
            rot_curr = np.empty((3, 3))
            try:
                curr_pos = arm.last_feedback.position_command
            except ValueError:
                curr_pos = arm.last_feedback.position

            xyz_curr = arm.FK(curr_pos)

            arm_xyz_target = xyz_curr + arm_inputs.delta_xyz

            r_x = R.from_euler('x', arm_inputs.delta_rot_xyz[0])
            r_y = R.from_euler('y', arm_inputs.delta_rot_xyz[1])
            r_z = R.from_euler('z', arm_inputs.delta_rot_xyz[2])
            arm_rot_target = R.from_matrix(rot_curr) * r_x * r_y * r_z

            curr_seed_ik = curr_pos
            curr_seed_ik[2] = abs(curr_seed_ik[2])

            joint_target = arm.ik_target_xyz_so3(
                curr_seed_ik,
                arm_xyz_target,
                arm_rot_target.as_matrix())

            arm_goal.add_waypoint(position=joint_target)
            return arm_goal

    def update(self, t_now: float, demo_input: 'Optional[DemoInputs]'=None):
        self.base.update(t_now)
        self.arm.update()

        self.base.send()
        self.arm.send()

        if self.state is TreadyControlState.EXIT:
            return False

        if demo_input is None:
            if t_now - self.mobile_last_fbk_t > 1.0:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, TreadyControlState.DISCONNECTED)
            return True

        self.mobile_last_fbk_t = t_now

        if demo_input.should_exit:
            self.transition_to(t_now, TreadyControlState.EXIT)
        elif demo_input.should_reset:
            self.transition_to(t_now, TreadyControlState.HOMING)

        if self.state is TreadyControlState.DISCONNECTED:
            self.mobile_last_fbk_t = t_now
            self.transition_to(t_now, TreadyControlState.TELEOP)
            return True

        elif self.state is TreadyControlState.HOMING:
            base_complete = not self.base.has_active_trajectory
            if base_complete and self.arm.at_goal:
                self.transition_to(t_now, TreadyControlState.TELEOP)
            return True

        elif self.state is TreadyControlState.TELEOP:
            desired_flipper_mode = demo_input.chassis.align_flippers
            if self.base.aligned_flipper_mode != desired_flipper_mode:
                self.base.aligned_flipper_mode = desired_flipper_mode
            elif self.base.is_aligning:
                # ignore new base commands while flippers are aligning
                pass
            else:
                chassis_vels, flipper_vels = self.compute_velocities(demo_input.chassis)
                self.base.set_chassis_vel_trajectory(t_now, self.base.chassis_ramp_time, chassis_vels)
                self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)

            arm_goal = self.compute_arm_goal(demo_input.arm)
            if arm_goal is not None: self.arm.set_goal(arm_goal)
            gripper_closed = self.arm.end_effector.state == 1.0
            if demo_input.arm.gripper_closed and not gripper_closed:
                self.arm.end_effector.close()
            elif not demo_input.arm.gripper_closed and gripper_closed:
                self.arm.end_effector.open()

            return True

        elif self.state is TreadyControlState.STARTUP:
            self.transition_to(t_now, TreadyControlState.HOMING)
            return True

    def transition_to(self, t_now: float, state: TreadyControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is TreadyControlState.HOMING:
            print("TRANSITIONING TO HOMING")
            self.base.set_color('magenta')
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(self.mobile_io, msg)

            # build trajectory
            flipper_home = np.array([-1, 1, 1, -1]) * np.deg2rad(15 + 45)
            self.base.chassis_traj = None
            self.base.set_flipper_trajectory(t_now, 5.0, p=flipper_home)

            g = hebi.arm.Goal(self.arm.size)
            g.add_waypoint(position=self.arm_home)
            self.arm.set_goal(g)

        elif state is TreadyControlState.TELEOP:
            print("TRANSITIONING TO TELEOP")
            base.clear_color()
            # Print Instructions
            set_mobile_io_mode_text(self.mobile_io, 2)

        elif state is TreadyControlState.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")
            self.base.chassis_traj = None
            self.base.flipper_traj = None
            self.base.set_color('blue')

        elif state is TreadyControlState.EXIT:
            print("TRANSITIONING TO EXIT")
            self.base.set_color("red")

            # unset mobileIO control config
            self.mobile_io.set_button_mode(6, 0)
            self.mobile_io.set_button_output(1, 0)
            self.mobile_io.set_button_output(8, 0)
            set_mobile_io_instructions(self.mobile_io, "Demo Stopped.", color="red")

        self.state = state


def config_mobile_io(m: 'MobileIO'):
    """Sets up mobileIO interface.

    Return a function that parses mobileIO feedback into the format
    expected by the Demo
    """

    # MobileIO Button Config
    reset_pose_btn = 1
    joined_flipper_btn = 6
    quit_btn = 8

    slider_flip1 = 3
    slider_flip2 = 4
    slider_flip3 = 5
    slider_flip4 = 6

    joy_fwd = 2
    joy_rot = 1

    joy_arm_x = 8
    joy_arm_y = 7

    z_up = 5
    z_down = 7

    arm_rot_ctrl = 2
    arm_lock = 4
    gripper_close = 3

    # set mobileIO control config
    m.set_led_color("blue")
    m.set_snap(slider_flip1, 0)
    m.set_snap(slider_flip2, 0)
    m.set_snap(slider_flip3, 0)
    m.set_snap(slider_flip4, 0)

    m.set_button_mode(joined_flipper_btn, 1)
    m.set_button_mode(arm_rot_ctrl, 1)
    m.set_button_mode(arm_lock, 1)
    m.set_button_mode(gripper_close, 1)

    m.set_button_output(reset_pose_btn, 1)
    m.set_button_output(quit_btn, 1)

    m.set_button_output(arm_rot_ctrl, 1)
    m.set_button_output(arm_lock, 1)

    def parse_mobile_io_feedback(m: 'MobileIO'):
        should_exit = m.get_button_state(quit_btn)
        should_reset = m.get_button_state(reset_pose_btn)

        # Chassis Control
        aligned_flipper_mode = m.get_button_state(joined_flipper_btn)

        # Flipper Control
        flip1 = m.get_axis_state(slider_flip1)
        flip2 = m.get_axis_state(slider_flip2)
        flip3 = m.get_axis_state(slider_flip3)
        flip4 = m.get_axis_state(slider_flip4)

        # Update instruction text if mode has just changed
        btn_val = m.get_button_diff(arm_rot_ctrl)
        if btn_val in [2, 3]:  # Value just changed (either ToOff or ToOn)
            set_mobile_io_mode_text(m, arm_rot_ctrl)

        if m.get_button_state(arm_rot_ctrl):
            base_vel_fwd = 0.0
            base_vel_rot = 0.0
            arm_drx = 0.5 * m.get_axis_state(joy_rot)
            arm_dry = 0.5 * m.get_axis_state(joy_fwd)
            arm_drz = 0.0
        else:
            base_vel_fwd = m.get_axis_state(joy_fwd)
            base_vel_rot = m.get_axis_state(joy_rot)
            arm_drx = 0.0
            arm_dry = 0.0
            arm_drz = 0.0

        arm_dx = 0.3 * m.get_axis_state(joy_arm_x)
        arm_dy = -0.3 * m.get_axis_state(joy_arm_y)

        arm_dz = 0.0
        if m.get_button_state(z_up):
            arm_dz = 0.1
        elif m.get_button_state(z_down):
            arm_dz = -0.1

        tready_inputs = TreadyInputs(
            ChassisVelocity(base_vel_fwd, base_vel_rot),
            [flip1, flip2, flip3, flip4],
            aligned_flipper_mode)

        arm_inputs = ArmInputs(
            [arm_dx, arm_dy, arm_dz],
            [arm_drx, arm_dry, arm_drz],
            m.get_button_state(arm_lock),
            m.get_button_state(gripper_close))

        return DemoInputs(should_exit, should_reset, tready_inputs, arm_inputs)

    return parse_mobile_io_feedback


if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    family = "Tready"

    # Base setup
    base = setup_base(lookup, family)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for mobileIO device to come online...')
    m = create_mobile_io(lookup, family, phone_name)
    while m is None:
        m = create_mobile_io(lookup, family, phone_name)
        print("Could not find mobileIO device, waiting...")
        sleep(0.5)

    print("mobileIO device found.")
    input_parser = config_mobile_io(m)

    arm = setup_arm_6dof(lookup, family)

    demo_controller = TreadyArmJoystickControl(m, base, arm)

    #######################
    ## Main Control Loop ##
    #######################

    should_continue = True
    while should_continue:
        t = time()
        demo_inputs = None
        if m.update(0.0):
            demo_inputs = input_parser(m)

        should_continue = demo_controller.update(t, demo_inputs)

    sys.exit(0)
