import sys
from collections import namedtuple
from time import time, sleep
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io


class TreadedBase:
    # FRAME CONVENTION:
    # ORIGIN = MID-POINT BETWEEN THE WHEELS
    # +X-AXIS = FORWARD
    # +Y-AXIS = LEFT
    # +Z-AXIS = UP

    #   Left  |   Right
    #   1     |    2
    #         |
    #         |
    #   3     |    4

    WHEEL_DIAMETER = 0.105
    WHEEL_BASE = 0.400

    WHEEL_RADIUS = WHEEL_DIAMETER / 2

    def __init__(self, group, chassis_ramp_time, flipper_ramp_time):
        self.group = group
        self.fbk = hebi.GroupFeedback(group.size)
        self.flipper_fbk = self.fbk.create_view([0, 1, 2, 3])
        self.wheel_fbk = self.fbk.create_view([4, 5, 6, 7])
        self.cmd = hebi.GroupCommand(group.size)
        self.flipper_cmd = self.cmd.create_view([0, 1, 2, 3])
        self.wheel_cmd = self.cmd.create_view([4, 5, 6, 7])

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.group.get_next_feedback(reuse_fbk=self.fbk)
        t_now = time()

        self.cmd.position = self.fbk.position
        self.t_prev = t_now
        self._aligned_flipper_mode = False
        self._aligning = False
        self.clear_trajectories()

    def has_active_trajectory(self, t_now):
        if self.chassis_traj is not None and t_now < self.chassis_traj.end_time:
            return True
        if self.flipper_traj is not None and t_now < self.flipper_traj.end_time:
            return True
        return False

    def clear_trajectories(self):
        self.chassis_traj = None
        self.flipper_traj = None

    @property
    def aligned_flipper_mode(self):
        return self._aligned_flipper_mode

    @property
    def aligning(self):
        return self._aligning

    @property
    def wheel_to_chassis_vel(self):
        wr = self.WHEEL_RADIUS / (self.WHEEL_BASE / 2)
        return np.array([
            [self.WHEEL_RADIUS, -self.WHEEL_RADIUS, self.WHEEL_RADIUS, -self.WHEEL_RADIUS],
            [0, 0, 0, 0],
            [wr, wr, wr, wr]
        ])

    @property
    def chassis_to_wheel_vel(self):
        return np.array([
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / (2 * self.WHEEL_RADIUS)],
        ])

    @property
    def flipper_aligned_front(self):
        return (self.flipper_cmd.position(0) + self.flipper_cmd.position(1)) < 0.01

    @property
    def flipper_aligned_back(self):
        return (self.flipper_cmd.position(2) + self.flipper_cmd.position(3)) < 0.01

    @property
    def flippers_aligned(self):
        return self.flipper_aligned_front and self.flipper_aligned_back

    @property
    def aligned_flipper_position(self):
        fmf = np.mean([self.flipper_cmd.position(0), -1 * self.flipper_cmd.position(1)])
        fmb = np.mean([self.flipper_cmd.position(2), -1 * self.flipper_cmd.position(3)])
        return np.array([fmf, -fmf, fmb, -fmb], dtype=np.float64)

    def update(self, t_now):
        dt = t_now - self.t_prev
        self.group.get_next_feedback(reuse_fbk=self.fbk)

        if self.flipper_traj is None and self.chassis_traj is None:
            self._aligning = False
            self.cmd.velocity = 0.0
        # finished aligning flippers
        elif self.aligning and self.flippers_aligned:
            self._aligning = False
        else:
            if self.chassis_traj is not None:
                # chassis update
                t = min(t_now, self.chassis_traj.end_time)
                [pos, vel, acc] = self.chassis_traj.get_state(t)
                self.wheel_cmd.velocity = self.chassis_to_wheel_vel @ vel
                self.wheel_cmd.position += self.wheel_cmd.velocity * dt

            if self.flipper_traj is not None:
                # flipper update
                t = min(t_now, self.flipper_traj.end_time)
                [pos, vel, _] = self.flipper_traj.get_state(t)
                self.flipper_cmd.velocity = vel

                if self.aligned_flipper_mode and self.flippers_aligned:
                    self.flipper_cmd.position = self.aligned_flipper_position

                self.flipper_cmd.position += vel * dt

        self.group.send_command(self.cmd)
        self.t_prev = t_now

    def set_flipper_trajectory(self, t_now, ramp_time, p=None, v=None):
        # Don't allow setting new trajectory while flippers are aligning
        if self.aligning:
            return

        times = [t_now, t_now + ramp_time]
        positions = np.empty((4, 2))
        velocities = np.empty((4, 2))
        efforts = np.empty((4, 2))

        if self.flipper_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            positions[:, 0], velocities[:, 0], efforts[:, 0] = self.flipper_traj.get_state(t)
        else:
            positions[:, 0] = base.flipper_fbk.position
            velocities[:, 0] = self.flipper_fbk.velocity
            efforts[:, 0] = self.flipper_fbk.effort

        positions[:, 1] = np.nan if p is None else p
        velocities[:, 1] = 0.0 if v is None else v
        efforts[:, 1] = 0.0

        self.flipper_traj = hebi.trajectory.create_trajectory(times, positions, velocities, efforts)

    def set_chassis_vel_trajectory(self, t_now, ramp_time, v):
        # Don't allow setting new trajectory while flippers are aligning
        if self.aligning:
            return

        times = [t_now, t_now + ramp_time]
        positions = np.empty((3, 2))
        velocities = np.empty((3, 2))
        efforts = np.empty((3, 2))

        if self.chassis_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            positions[:, 0], velocities[:, 0], efforts[:, 0] = self.chassis_traj.get_state(t)
        else:
            positions[:, 0] = 0.0
            velocities[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.velocity
            efforts[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.effort

        positions[:, 1] = np.nan
        velocities[:, 1] = v
        efforts[:, 1] = 0.0

        self.chassis_traj = hebi.trajectory.create_trajectory(times, positions, velocities, efforts)

    def align_flippers(self, t_now):
        self._aligned_flipper_mode = True
        self._aligning = True
        self.base.clear_trajectories()
        self.base.set_flipper_trajectory(t_now, 3.0, p=self.aligned_flipper_position)

    def unlock_flippers(self):
        self._aligned_flipper_mode = False
        self._aligning = False

    def set_color(self, color):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.clear()
        self.group.send_command(color_cmd)


class DemoState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    STOPPED = auto()
    EXIT = auto()


ChassisVelocity = namedtuple('ChassisVelocity', ['x', 'rz'])


class TreadyInputs:
    def __init__(self, chassis, flippers, align_flippers):
        self.chassis = chassis
        self.flippers = flippers
        self.align_flippers = align_flippers


class ArmInputs:
    def __init__(self, phone_pos, phone_rot, locked, active):
        self.phone_pos = phone_pos
        self.phone_rot = phone_rot
        self.locked = locked
        self.active = active


class DemoInputs:
    def __init(self, chassis_inputs, arm_inputs):
        self.chassis = chassis_inputs
        self.arm = arm_inputs


class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, mobile_io, base: TreadedBase, arm: hebi.arm.Arm):
        self.state = DemoState(DemoState.STARTUP)
        self.mobile_io = mobile_io
        self.base = base
        self.arm = arm

        self.SPEED_MAX_LIN = 0.125  # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / base.WHEEL_BASE  # rad/s

        self.arm_xyz_home = [0.34, 0.0, 0.23]
        self.arm_rot_home = R.from_euler('z', np.pi) * R.from_euler('y', np.pi / 2)
        self.arm_rot_home = self.arm_rot_home.as_matrix()

        self.arm_seed_ik = np.array([0.25, -1.0, 0, -0.75, 0])
        self.arm_home = self.arm.ik_target_xyz_so3(
            self.arm_seed_ik,
            self.arm_xyz_home,
            self.arm_rot_home)

    def compute_arm_goal(self, t_now, inputs):
        # Exaggerate Z-Axis by 2x, X-Y are 1-to-1.
        xyz_scale = np.array([1.0, 1.0, 2.0])

        phone_xyz = inputs.arm.phone_pos
        phone_rot = inputs.arm.phone_rot

        arm_xyz_target = self.arm_xyz_home

        if not inputs.arm.active:
            # update phone "zero"
            self.phone_xyz_home = phone_xyz

        arm_goal = hebi.arm.Goal(self.arm.size)
        if inputs.arm.locked:
            arm_goal.add_waypoint(positions=self.arm_home)
            return arm_goal
        elif inputs.arm.active:
            phone_offset = phone_xyz - self.phone_xyz_home
            rot_mat = self.phone_rot_home.as_matrix()
            arm_xyz_target += xyz_scale * (rot_mat.T @ phone_offset)
            arm_rot_target = rot_mat.T @ phone_rot @ self.arm_rot_home

            joint_target = self.arm.ik_target_xyz_so3(
                self.arm_seed_ik,
                arm_xyz_target,
                arm_rot_target)

            arm_goal.add_waypoint(positions=joint_target)
            return arm_goal

        return None

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

        vel_x = self.SPEED_MAX_LIN * inputs.chassis.x
        vel_y = 0
        vel_rot = self.SPEED_MAX_ROT * inputs.chassis.rz

        chassis_vels = [vel_x, vel_y, vel_rot]

        return chassis_vels, flipper_vels

    def update(self, t_now, demo_input=None):
        self.base.update(t_now)
        self.arm.update()

        if demo_input is not None:
            self.mobile_last_fbk_t = t_now
        elif t_now - self.mobile_last_fbk_t > 1.0:
            print("mobileIO timeout, disabling motion")
            self.transition_to(t_now, DemoState.STOPPED)

        if self.state is DemoState.STARTUP:
            self.transition_to(t_now, DemoState.HOMING)

        elif self.state is DemoState.HOMING:
            base_complete = not base.has_active_trajectory(t_now)
            if base_complete and self.arm.at_goal:
                self.phone_xyz_home = demo_input.arm.phone_pos
                self.phone_rot_home = demo_input.arm.phone_rot
                self.transition_to(t_now, DemoState.TELEOP)

        elif self.state is DemoState.TELEOP:
            if demo_input.chassis.align_flippers and self.base.aligning:
                pass
            elif demo_input.chassis.align_flippers and not self.base.flippers_aligned:
                self.base.align_flippers(t_now)
            else:
                chassis_vels, flipper_vels = self.compute_velocities(t_now, demo_input)
                self.base.set_chassis_vel_trajectory(t_now, self.chassis_ramp_time, chassis_vels)
                self.base.set_flipper_trajectory(t_now, self.flipper_ramp_time, v=flipper_vels)

            arm_goal = self.compute_arm_goal(t_now, demo_input)
            arm.set_goal(arm_goal)

        elif self.state is DemoState.STOPPED:
            if demo_input is not None:
                self.mobile_last_fbk_t = t_now
                self.transition_to(t_now, DemoState.TELEOP)

        elif self.state is DemoState.EXIT:
            return False

        return True

    def transition_to(self, t_now, state):
        if state is DemoState.HOMING:
            print("TRANSITIONING TO HOMING")
            base.set_color('magenta')
            self.mobile_io.clear_text()
            msg = 'Robot Homing Sequence\nPlease wait...'
            self.mobile_io.set_text(msg)

            # build trajectory
            flipper_home = np.deg2rad([-15, 15, 15, -15])
            self.base.clear_trajectories()
            self.base.set_flipper_trajectory(t_now, 5.0, p=flipper_home)

            g = hebi.arm.Goal(self.arm.size)
            g.add_waypoint(position=self.arm_home)
            self.arm.set_goal(g)

        elif state is DemoState.TELEOP:
            print("TRANSITIONING TO TELEOP")
            base.clear_color()
            # Print Instructions
            self.mobile_io.clear_text()
            instructions = 'Robot Ready to Control\nB1: Reset\nB6: Joined Flipper\nB8 - Quit'
            self.mobile_io.set_text(instructions)
            self.mobile_io.set_led_color("green")

        elif state is DemoState.STOPPED:
            print("TRANSITIONING TO STOPPED")
            base.clear_trajectories()
            base.set_color('blue')

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


if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    # Base setup
    base_family = "Tready"
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # Create base group
    group = lookup.get_group_from_names([base_family], flipper_names + wheel_names)
    if group is None:
        raise RuntimeError(f"Could not find modules: {flipper_names + wheel_names} in family '{base_family}'")
    load_gains(group, "gains/tready.xml")

    base = TreadedBase(group, 0.25, 0.33)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for mobileIO device to come online...')
    m = create_mobile_io(lookup, base_family, phone_name)
    while m is None:
        m = create_mobile_io(lookup, base_family, phone_name)
        print("Could not find mobileIO device, waiting...")
        sleep(0.5)

    print("mobileIO device found.")

    # arm setup
    arm = hebi.arm.create(
        [base_family],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_wrist1', 'J5_wrist2'],
        hrdf_file='hrdf/tready-arm',
        lookup=lookup)

    mirror_group = lookup.get_group_from_names([base_family], 'J2B_shoulder1')
    arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    arm.load_gains('gains/tready-arm-gains')

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

    # set mobileIO control config
    m.set_led_color("blue")
    m.set_snap(slider_flip1, 0)
    m.set_snap(slider_flip2, 0)
    m.set_snap(slider_flip3, 0)
    m.set_snap(slider_flip4, 0)

    m.set_button_mode(joined_flipper_btn, 1)
    m.set_button_mode(arm_enable, 1)
    m.set_button_mode(arm_lock, 1)

    m.set_button_output(reset_pose_btn, 1)
    m.set_button_output(quit_btn, 1)

    m.set_button_output(arm_enable, 1)
    m.set_button_output(arm_lock, 1)

    def parse_mobile_io_feedback(m):
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

        arm_inputs = ArmInputs(
            m.position,
            R.from_quat(m.orientation).as_matrix(),
            m.get_button_state(arm_lock),
            m.get_button_state(arm_enable))

        return DemoInputs(tready_inputs, arm_inputs)

    demo_controller = TreadyControl(m, base, arm)

    #######################
    ## Main Control Loop ##
    #######################

    while True:
        t = time()

        demo_inputs = None
        if m.update(0.0):
            demo_inputs = parse_mobile_io_feedback(m)

        if m.get_button_state(quit_btn) == 1:
            demo_controller.transition_to(t, DemoState.EXIT)
        elif m.get_button_state(reset_pose_btn) == 1:
            demo_controller.transition_to(t, DemoState.HOMING)

        if not demo_controller.update(t, demo_inputs):
            break

    sys.exit(0)
