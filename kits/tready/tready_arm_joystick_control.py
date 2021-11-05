import sys
import os
from collections import namedtuple
from time import time, sleep
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

from tready import TreadedBase


def set_instructions(mobile_io, mode_button):
    if mobile_io.get_button_state(mode_button):
        mode_text = 'Arm Rot. Mode'
        m.set_led_color("yellow")
    else:
        mode_text = 'Base Drive Mode'
        m.set_led_color("green")

    instructions = ('Robot Ready to Control\n'
                    'B1: Reset\n'
                    'B2: {}\n'
                    'B3: Toggle Gripper\n'
                    'B4: Arm Home\n'
                    'B5/7: Arm Up/Down\n'
                    'B6: Joined Flipper\n'
                    'B8 - Quit')
    mobile_io.clear_text()
    mobile_io.set_text(instructions.format(mode_text))


class DemoState(Enum):
    STARTUP = auto()
    HOMING = auto()
    ALIGNING = auto()
    TELEOP = auto()
    STOPPED = auto()
    EXIT = auto()


ChassisVelocity = namedtuple('ChassisVelocity', ['x', 'rz'])
TreadyInputs = namedtuple('TreadyInputs', ['base_motion', 'flippers', 'align_flippers'])
ArmInputs = namedtuple('ArmInputs', ['delta_xyz', 'delta_rot_xyz', 'locked', 'gripper_closed'])
DemoInputs = namedtuple('DemoInputs', ['should_exit', 'should_reset', 'chassis', 'arm'])


class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, mobile_io, base: TreadedBase, arm: hebi.arm.Arm):
        self.state = DemoState(DemoState.STARTUP)
        self.mobile_io = mobile_io
        self.base = base
        self.arm = arm

        self.SPEED_MAX_LIN = 0.15  # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / base.WHEEL_BASE  # rad/s

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


    def compute_arm_goal(self, t_now, inputs):
        arm_goal = hebi.arm.Goal(self.arm.size)
        if inputs.arm.locked:
            arm_goal.add_waypoint(t=3.0, position=self.arm_home)
            return arm_goal
        else:
            rot_curr = np.empty((3,3))
            try:
                xyz_curr = self.arm.FK(self.arm.last_feedback.position_command, orientation_out=rot_curr)
            except ValueError:
                xyz_curr = self.arm.FK(self.arm.last_feedback.position, orientation_out=rot_curr)

            arm_xyz_target = xyz_curr + inputs.arm.delta_xyz

            r_x = R.from_euler('x', inputs.arm.delta_rot_xyz[0])
            r_y = R.from_euler('y', inputs.arm.delta_rot_xyz[1])
            r_z = R.from_euler('z', inputs.arm.delta_rot_xyz[2])
            arm_rot_target = R.from_matrix(rot_curr) * r_x * r_y * r_z

            curr_seed_ik = self.arm.last_feedback.position_command
            curr_seed_ik[2] = abs(curr_seed_ik[2])

            joint_target = self.arm.ik_target_xyz_so3(
                curr_seed_ik,
                arm_xyz_target,
                arm_rot_target.as_matrix())

            arm_goal.add_waypoint(position=joint_target)
            return arm_goal

    def compute_velocities(self, inputs):
        # Flipper Control
        [flip1, flip2, flip3, flip4] = inputs.chassis.flippers

        if inputs.chassis.align_flippers:
            f_vel1 = max(abs(flip1), abs(flip2)) * np.sign(flip1 + flip2) * self.FLIPPER_VEL_SCALE
            f_vel2 = -f_vel1
            f_vel3 = max(abs(flip3), abs(flip4)) * np.sign(flip3 + flip4) * self.FLIPPER_VEL_SCALE
            f_vel4 = -f_vel3

        else:
            f_vel1 = flip1 * self.FLIPPER_VEL_SCALE
            f_vel2 = -1 * flip2 * self.FLIPPER_VEL_SCALE
            f_vel3 = flip3 * self.FLIPPER_VEL_SCALE
            f_vel4 = -1 * flip4 * self.FLIPPER_VEL_SCALE

        flipper_vels = [f_vel1, f_vel2, f_vel3, f_vel4]

        # Mobile Base Control

        vel_x = self.SPEED_MAX_LIN * inputs.chassis.base_motion.x
        vel_y = 0
        vel_rot = self.SPEED_MAX_ROT * inputs.chassis.base_motion.rz

        chassis_vels = [vel_x, vel_y, vel_rot]

        return chassis_vels, flipper_vels

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
                    self.transition_to(t_now, DemoState.TELEOP)
                return True

            elif self.state is DemoState.ALIGNING:
                should_align_flippers = demo_input.chassis.align_flippers
                if self.base.flippers_aligned or not should_align_flippers:
                    self.transition_to(t_now, DemoState.TELEOP)

                arm_goal = self.compute_arm_goal(t_now, demo_input)
                if arm_goal is not None:
                    self.arm.set_goal(arm_goal)
                gripper_closed = self.arm.end_effector.state == 1.0
                if demo_input.arm.gripper_closed and not gripper_closed:
                    self.arm.end_effector.close()
                elif not demo_input.arm.gripper_closed and gripper_closed:
                    self.arm.end_effector.open()

                return True

            elif self.state is DemoState.TELEOP:
                should_align_flippers = demo_input.chassis.align_flippers
                if should_align_flippers and not self.base.aligned_flipper_mode:
                    self.transition_to(t_now, DemoState.ALIGNING)
                elif not should_align_flippers and self.base.aligned_flipper_mode:
                    self.base.unlock_flippers()
                else:
                    chassis_vels, flipper_vels = self.compute_velocities(demo_input)
                    self.base.set_chassis_vel_trajectory(t_now, self.base.chassis_ramp_time, chassis_vels)
                    self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)

                arm_goal = self.compute_arm_goal(t_now, demo_input)
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
            self.mobile_io.clear_text()
            msg = 'Robot Homing Sequence\nPlease wait...'
            self.mobile_io.set_text(msg)

            # build trajectory
            flipper_home = np.array([-1, 1, 1, -1]) * np.deg2rad(15 + 45)
            self.base.chassis_traj = None
            self.base.set_flipper_trajectory(t_now, 5.0, p=flipper_home)

            g = hebi.arm.Goal(self.arm.size)
            g.add_waypoint(position=self.arm_home)
            self.arm.set_goal(g)

        elif state is DemoState.ALIGNING:
            print("TRANSITIONING TO ALIGNING")
            self.base.align_flippers(t_now)

        elif state is DemoState.TELEOP:
            print("TRANSITIONING TO TELEOP")
            base.clear_color()
            # Print Instructions
            set_instructions(self.mobile_io, 2)
            self.mobile_io.set_led_color("green")

        elif state is DemoState.STOPPED:
            print("TRANSITIONING TO STOPPED")
            self.base.chassis_traj = None
            self.base.flipper_traj = None
            self.base.set_color('blue')

        elif state is DemoState.EXIT:
            print("TRANSITIONING TO EXIT")
            # unset mobileIO control config
            self.mobile_io.set_led_color("red")
            self.base.set_color("red")

            self.mobile_io.set_button_mode(6, 0)
            self.mobile_io.set_button_output(1, 0)
            self.mobile_io.set_button_output(8, 0)
            self.mobile_io.clear_text()
            self.mobile_io.set_text('Demo Stopped.')

        self.state = state


def load_gains(group, gains_file):
    gains_command = hebi.GroupCommand(group.size)
    sleep(0.1)

    try:
        gains_command.read_gains(gains_file)
    except Exception as e:
        print(f'Warning - Could not load gains: {e}')
        return

    # Send gains multiple times
    for i in range(3):
        group.send_command(gains_command)
        sleep(0.1)


def setup_base(lookup, base_family):
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # Create base group
    group = lookup.get_group_from_names([base_family], wheel_names + flipper_names)
    if group is None:
        raise RuntimeError(f"Could not find modules: {wheel_names + flipper_names} in family '{base_family}'")

    root_dir, _ = os.path.split(__file__)
    load_gains(group, os.path.join(root_dir, "gains/r-tready-gains.xml"))

    return TreadedBase(group, 0.25, 0.33)


def setup_arm(lookup, family):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=os.path.join(root_dir, 'hrdf/tready-arm.hrdf'),
        lookup=lookup)

    mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder1'])
    arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    arm.load_gains(os.path.join(root_dir, 'gains/tready-arm-gains.xml'))
    return arm


def setup_arm_6dof(lookup, family):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=os.path.join(root_dir, 'hrdf/tready-arm-A2240-06G.hrdf'),
        lookup=lookup)

    arm.load_gains(os.path.join(root_dir, 'gains/A-2240-06.xml'))

    # Add the gripper
    gripper = hebi.arm.Gripper(lookup.get_group_from_names([family], ['gripperSpool']), -5, 1)
    gripper.load_gains(os.path.join(root_dir, 'gains/gripper_spool_gains.xml'))
    arm.set_end_effector(gripper)

    return arm


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

    def parse_mobile_io_feedback(m):
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
        if btn_val in [2, 3]: # Value just changed (either ToOff or ToOn)
            set_instructions(m, arm_rot_ctrl)

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

    demo_controller = TreadyControl(m, base, arm)

    #######################
    ## Main Control Loop ##
    #######################

    while True:
        t = time()
        demo_inputs = None
        if m.update(0.0):
            demo_inputs = input_parser(m)

        should_continue = demo_controller.update(t, demo_inputs)
        if not should_continue:
            break

    sys.exit(0)
