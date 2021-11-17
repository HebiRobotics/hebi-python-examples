import sys
import os
from collections import namedtuple
from time import time, sleep
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

from tready import TreadedBase, TreadyControl, ChassisVelocity, TreadyInputs
from tready_utils import set_mobile_io_instructions, setup_base, setup_arm_6dof


class DemoState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    STOPPED = auto()
    EXIT = auto()


ArmInputs = namedtuple('ArmInputs', ['phone_pos', 'phone_rot', 'locked', 'active', 'gripper_closed'])
DemoInputs = namedtuple('DemoInputs', ['should_exit', 'should_reset', 'chassis', 'arm'])


class TreadyArmIOControl(TreadyControl):

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, mobile_io, base: TreadedBase, arm: hebi.arm.Arm):
        super(TreadyArmIOControl, self).__init__(mobile_io, base)

        self.arm = arm

        # 5 DoF home
        #self.arm_xyz_home = [0.34, 0.0, 0.23]
        # 6 DoF home
        self.arm_xyz_home = [0.4, 0.0, 0.0]
        self.arm_rot_home = R.from_euler('z', np.pi / 2) * R.from_euler('x', np.pi)
        self.arm_rot_home = self.arm_rot_home.as_matrix()

        # 5 DoF seed
        #self.arm_seed_ik = np.array([0.25, -1.0, 0, -0.75, 0])
        # 6 DoF seed
        self.arm_seed_ik = np.array([0, 0.5, 2, 3, -1.5, 0])
        self.arm_home = self.arm.ik_target_xyz_so3(
            self.arm_seed_ik,
            self.arm_xyz_home,
            self.arm_rot_home)

    def compute_arm_goal(self, arm_inputs):
        xyz_scale = np.array([1.0, 1.0, 1.0])

        phone_xyz = arm_inputs.phone_pos
        phone_rot = arm_inputs.phone_rot

        if arm_inputs.locked or not arm_inputs.active:
            # update phone "zero"
            self.phone_xyz_home = phone_xyz

        arm_goal = hebi.arm.Goal(self.arm.size)
        if arm_inputs.arm.locked:
            arm_goal.add_waypoint(position=self.arm_home)
            return arm_goal
        elif arm_inputs.arm.active:
            phone_offset = phone_xyz - self.phone_xyz_home
            rot_mat = self.phone_rot_home
            arm_xyz_target = self.arm_xyz_home + xyz_scale * (rot_mat.T @ phone_offset)
            arm_rot_target = rot_mat.T @ phone_rot @ self.arm_rot_home

            joint_target = self.arm.ik_target_xyz_so3(
                self.arm_seed_ik,
                arm_xyz_target,
                arm_rot_target)

            arm_goal.add_waypoint(position=joint_target)
            return arm_goal

        return None

    def update(self, t_now: float, demo_input=None):
        self.base.update(t_now)
        self.arm.update()

        self.base.send()
        self.arm.send()

        if self.state is DemoState.EXIT:
            return False

        if demo_input is None:
            if t_now - self.mobile_last_fbk_t > 1.0:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, DemoState.STOPPED)
            return True
        else:
            self.mobile_last_fbk_t = t_now

            if demo_input.should_exit:
                self.transition_to(t_now, DemoState.EXIT)
            elif demo_input.should_reset:
                self.transition_to(t_now, DemoState.HOMING)

            if self.state is DemoState.STOPPED:
                self.mobile_last_fbk_t = t_now
                self.transition_to(t_now, DemoState.TELEOP)
                return True

            elif self.state is DemoState.HOMING:
                base_complete = not self.base.has_active_trajectory(t_now)
                if base_complete and self.arm.at_goal:
                    self.phone_xyz_home = demo_input.arm.phone_pos
                    self.phone_rot_home = demo_input.arm.phone_rot
                    self.transition_to(t_now, DemoState.TELEOP)
                return True

            elif self.state is DemoState.TELEOP:
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
                if arm_goal is not None:
                    self.arm.set_goal(arm_goal)
                gripper_closed = self.arm.end_effector.state == 1.0
                if demo_input.arm.gripper_closed and not gripper_closed:
                    self.arm.end_effector.close()
                elif not demo_input.arm.gripper_closed and gripper_closed:
                    self.arm.end_effector.open()

                return True

            elif self.state is DemoState.STARTUP:
                self.transition_to(t_now, DemoState.HOMING)
                return True

    def transition_to(self, t_now, state):
        # self transitions are noop
        if state == self.state:
            return

        if state is DemoState.HOMING:
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

        elif state is DemoState.TELEOP:
            print("TRANSITIONING TO TELEOP")
            base.clear_color()
            # Print Instructions
            instructions = ('Robot Ready to Control\n'
                            'B1: Reset\n'
                            'B2: Arm Motion Enable\n'
                            'B4: Arm Lock\n'
                            'B5: Close Gripper\n'
                            'B6: Joined Flipper\n'
                            'B8 - Quit')
            set_mobile_io_instructions(self.mobile_io, instructions, color='green')

        elif state is DemoState.STOPPED:
            print("TRANSITIONING TO STOPPED")
            self.base.chassis_traj = None
            self.base.flipper_traj = None
            self.base.set_color('blue')

        elif state is DemoState.EXIT:
            print("TRANSITIONING TO EXIT")
            self.base.set_color("red")

            # unset mobileIO control config
            self.mobile_io.set_button_mode(6, 0)
            self.mobile_io.set_button_output(1, 0)
            self.mobile_io.set_button_output(8, 0)
            set_mobile_io_instructions(self.mobile_io, 'Demo Stopped.', color='red')

        self.state = state


def config_mobile_io(m):
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

    arm_enable = 2
    arm_lock = 4
    gripper_close = 5

    # set mobileIO control config
    m.set_led_color("blue")
    m.set_snap(slider_flip1, 0)
    m.set_snap(slider_flip2, 0)
    m.set_snap(slider_flip3, 0)
    m.set_snap(slider_flip4, 0)

    m.set_button_mode(joined_flipper_btn, 1)
    m.set_button_mode(arm_enable, 1)
    m.set_button_mode(arm_lock, 1)
    m.set_button_mode(gripper_close, 1)

    m.set_button_output(reset_pose_btn, 1)
    m.set_button_output(quit_btn, 1)

    m.set_button_output(arm_enable, 1)
    m.set_button_output(arm_lock, 1)

    def parse_mobile_io_feedback(m):
        should_exit = m.get_button_state(quit_btn)
        should_reset = m.get_button_state(reset_pose_btn)
        # Chassis Control
        aligned_flipper_mode = m.get_button_state(joined_flipper_btn)
        joy_vel_fwd = m.get_axis_state(joy_fwd)
        joy_vel_rot = m.get_axis_state(joy_rot)

        # Flipper Control
        flip1 = m.get_axis_state(slider_flip1)
        flip2 = m.get_axis_state(slider_flip2)
        flip3 = m.get_axis_state(slider_flip3)
        flip4 = m.get_axis_state(slider_flip4)

        tready_inputs = TreadyInputs(
            ChassisVelocity(joy_vel_fwd, joy_vel_rot),
            [flip1, flip2, flip3, flip4],
            aligned_flipper_mode)

        try:
            # reorder quaternion components
            wxyz = m.orientation
            xyzw = [*wxyz[1:4], wxyz[0]]
            rotation = R.from_quat(xyzw).as_matrix()
        except ValueError as e:
            print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
            rotation = np.eye(3)

        arm_inputs = ArmInputs(
            np.copy(m.position),
            rotation,
            m.get_button_state(arm_lock),
            m.get_button_state(arm_enable),
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

    demo_controller = TreadyArmIOControl(m, base, arm)

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
