import sys
from time import time, sleep
from enum import Enum, auto

import numpy as np

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

        times = np.array([0.0, chassis_ramp_time]) + t_now
        self.chassis_traj = hebi.trajectory.create_trajectory(times, np.zeros((3, 2)))
        times = np.array([0.0, flipper_ramp_time]) + t_now
        self.flipper_traj = hebi.trajectory.create_trajectory(times, np.zeros((4, 2)))

        self.cmd.position = self.fbk.position
        self.t_prev = t_now

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
    def flipper_mean_front(self):
        return np.mean([self.flipper_cmd.position(1), -1 * self.flipper_cmd.position(2)])

    @property
    def flipper_mean_back(self):
        return np.mean([self.flipper_cmd.position(3), -1 * self.flipper_cmd.position(4)])

    @property
    def flipper_aligned_front(self):
        return (self.flipper_cmd.position(1) + self.flipper_cmd.position(2)) < 0.01

    @property
    def flipper_aligned_back(self):
        return (self.flipper_cmd.position(3) + self.flipper_cmd.position(4)) < 0.01

    @property
    def flippers_aligned(self):
        return flipper_aligned_front and flipper_aligned_back

    @property
    def aligned_flipper_position(self):
        fmf = self.flipper_mean_front
        fmb = self.flipper_mean_back
        return np.array([fmf, -fmf, fmb, -fmb], dtype=np.float64)

    def update(self, t_now):
        dt = t_now - self.t_prev

        if self.flipper_traj is None and self.chassis_traj is None:
            self.cmd.velocity = 0.0
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
                [pos, vel, eff] = self.flipper_traj.get_state(t)
                self.flipper_cmd.velocity = vel

                if self.joined_flipper_mode and self.flippers_aligned:
                    self.flipper_cmd.position = self.aligned_flipper_position

                self.flipper_cmd.position += vel * dt

        self.group.send_command(self.cmd)
        self.t_prev = t_now

    def set_flipper_trajectory(self, t_now, ramp_time, p=None, v=None):
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

    def set_color(self, color):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.clear()
        self.group.send_command(color_cmd)


class TreadyState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    ALIGN_FLIPPERS = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, base, mobile_io):
        self.state = TreadyState(TreadyState.STARTUP)
        self.base = base
        self.mobile_io = mobile_io
        self.SPEED_MAX_LIN = 0.125  # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / base.WHEEL_BASE  # rad/s

    def process_mobile_input(self, t_now):
        reset_pose_btn = 1
        joined_flipper_btn = 6
        quit_btn = 8

        slider_flip1 = 3
        slider_flip2 = 4
        slider_flip3 = 5
        slider_flip4 = 6

        joy_fwd = 2
        joy_rot = 1

        aligned_flipper_mode = self.mobile_io.get_button_state(joined_flipper_btn)

        ret_val = {
            'change_state': None,
            'flipper_pos': None,
            'flipper_vel': None,
            'chassis_vel': None
        }

        if self.mobile_io.get_button_state(quit_btn) == 1:
            ret_val['change_state'] = TreadyState.EXIT
            return ret_val
        elif self.mobile_io.get_button_state(reset_pose_btn) == 1:
            ret_val['change_state'] = TreadyState.HOMING
            return ret_val
        elif aligned_flipper_mode and not self.base.flippers_aligned:
            ret_val['change_state'] = TreadyState.ALIGN_FLIPPERS
            return ret_val

        # Flipper Control
        flip1 = self.mobile_io.get_axis_state(slider_flip1)
        flip2 = self.mobile_io.get_axis_state(slider_flip2)
        flip3 = self.mobile_io.get_axis_state(slider_flip3)
        flip4 = self.mobile_io.get_axis_state(slider_flip4)

        if aligned_flipper_mode:
            f_vel1 = max(abs(flip1), abs(flip2)) * np.sign(flip1 + flip2) * self.FLIPPER_VEL_SCALE
            f_vel2 = -f_vel1
            f_vel3 = max(abs(flip3), abs(flip4)) * np.sign(flip3 + flip4) * self.FLIPPER_VEL_SCALE
            f_vel4 = -f_vel3

            ret_val['flipper_pos'] = np.repeat(np.nan, 4)

        else:
            f_vel1 = flip1 * self.FLIPPER_VEL_SCALE
            f_vel2 = -1 * flip2 * self.FLIPPER_VEL_SCALE
            f_vel3 = flip3 * self.FLIPPER_VEL_SCALE
            f_vel4 = -1 * flip4 * self.FLIPPER_VEL_SCALE

        ret_val['flipper_vel'] = [f_vel1, f_vel2, f_vel3, f_vel4]

        # Mobile Base Control
        joy_vel_fwd = self.mobile_io.get_axis_state(joy_fwd)
        joy_vel_rot = self.mobile_io.get_axis_state(joy_rot)

        vel_x = self.SPEED_MAX_LIN * joy_vel_fwd
        vel_y = 0
        vel_rot = self.SPEED_MAX_ROT * joy_vel_rot

        ret_val['chassis_vel'] = [vel_x, vel_y, vel_rot]

        return None, chassis_vels, flipper_vels

    def update(self, t_now):
        has_mobile_feedback = self.mobile_io.update(0.0)
        if has_mobile_feedback:
            self.mobile_last_fbk_t = t_now
        self.base.update(t_now)
        if self.state is TreadyState.STARTUP:
            # set mobileIO control config
            self.mobile_io.set_led_color("blue")
            self.mobile_io.set_snap(3, 0)
            self.mobile_io.set_snap(4, 0)
            self.mobile_io.set_snap(5, 0)
            self.mobile_io.set_snap(6, 0)

            self.mobile_io.set_button_mode(6, 1)

            self.mobile_io.set_button_output(1, 1)
            self.mobile_io.set_button_output(8, 1)
            self.transition_to(t_now, TreadyState.HOMING)

        elif self.state is TreadyState.HOMING:
            if not base.has_active_trajectory(t_now):
                self.transition_to(t_now, TreadyState.TELEOP)

        elif self.state is TreadyState.ALIGN_FLIPPERS:
            if not base.has_active_trajectory(t_now):
                self.transition_to(t_now, TreadyState.TELEOP)

        elif self.state is TreadyState.TELEOP:
            if not has_mobile_feedback:
                if t_now - self.mobile_last_fbk_t > 1.0:
                    print("mobileIO timeout, disabling motion")
                    self.transition_to(t_now, TreadyState.DISCONNECTED)
            else:
                next_state, chassis_vels, flipper_vels = self.process_mobile_input(t_now)
                if next_state is not None:
                    self.transition_to(t_now, next_state)
                else:
                    self.base.set_chassis_vel_trajectory(t_now, self.chassis_ramp_time, chassis_vels)
                    self.base.set_flipper_trajectory(t_now, self.flipper_ramp_time, v=flipper_vels)

        elif self.state is TreadyState.DISCONNECTED:
            if has_mobile_feedback:
                self.mobile_last_fbk_t = t_now
                self.transition_to(t_now, TreadyState.TELEOP)

        elif self.state is TreadyState.EXIT:
            return False

        return True

    def transition_to(self, t_now, state):
        if state is TreadyState.HOMING:
            print("TRANSITIONING TO HOMING")
            base.set_color('magenta')
            m.clear_text()
            msg = 'Robot Homing Sequence\nPlease wait...'
            m.set_text(msg)

            # build trajectory
            flipper_home = np.deg2rad([-15, 15, 15, -15])
            times = np.array([0.0, 5.0]) + t_now

            if self.state is TreadyState.STARTUP:
                flipper_positions = np.empty((4, 2))
                flipper_positions[:, 0] = base.flipper_fbk.position
                flipper_positions[:, 1] =
                self.flipper_pos_traj = hebi.trajectory.create_trajectory(times, flipper_positions)
            else:
                base.set_flipper_trajectory(t_now, 5.0, p=flipper_home)

        elif state is TreadyState.ALIGN_FLIPPERS:
            print("ALIGNING FLIPPERS")
            m.clear_text()
            msg = 'Aligning Flippers\nPlease wait...'
            m.set_text(msg)
            self.base.clear_trajectories()
            # this sets flipper_pos_traj, so flippers align
            pos = [self.base.flipper_fbk.position_command, self.base.aligned_flipper_position]
            self.base.set_flipper_trajectory(t_now, 3.0, p=pos)

        elif state is TreadyState.TELEOP:
            print("TRANSITIONING TO TELEOP")
            base.clear_color()
            # Print Instructions
            m.clear_text()
            instructions = 'Robot Ready to Control\nB1: Reset\nB6: Joined Flipper\nB8 - Quit'
            m.set_text(instructions)
            m.set_led_color("green")

        elif state is TreadyState.DISCONNECTED:
            print("TRANSITIONING TO DISCONNECTED")
            base.clear_trajectories()
            base.set_color('blue')

        elif state is TreadyState.EXIT:
            print("TRANSITIONING TO EXIT")
            # unset mobileIO control config
            self.mobile_io.set_led_color("red")
            self.base.set_color("red")

            self.mobile_io.set_button_mode(6, 0)
            self.mobile_io.set_button_output(1, 0)
            self.mobile_io.set_button_output(8, 0)
            m.clear_text()
            m.set_text('Demo Stopped.')

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

    # Create group
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

    #######################
    ## Main Control Loop ##
    #######################
    demo_controller = TreadyControl(base, m)

    while demo_controller.update(time()):
        pass
    sys.exit(0)
