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

        self.joined_flipper_mode = False

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.group.get_next_feedback(reuse_fbk=self.fbk)
        t_now = time()

        times = np.array([0.0, chassis_ramp_time]) + t_now
        self.chassis_traj = hebi.trajectory.create_trajectory(times, np.zeros((3, 2)))
        times = np.array([0.0, flipper_ramp_time]) + t_now
        self.flipper_traj = hebi.trajectory.create_trajectory(times, np.zeros((4, 2)))
        self.flipper_pos_traj = None

        self.cmd.position = self.fbk.position
        self.t_prev = t_now

    def trajectory_complete(self, t_now):
        if self.chassis_traj is not None and t_now < self.chassis_traj.end_time:
            return False
        if self.flipper_traj is not None and t_now < self.flipper_traj.end_time:
            return False
        if self.flipper_pos_traj is not None and t_now < self.flipper_pos_traj.end_time:
            return False
        return True

    def cancel_trajectory(self):
        self.chassis_traj = None
        self.flipper_traj = None
        self.flipper_pos_traj = None

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

        # switch to position trajectory to align flippers if needed
        if self.joined_flipper_mode and not self.flippers_aligned:
            # this sets flipper_pos_traj, so flippers align
            pos = np.array([self.flipper_cmd.position, self.aligned_flipper_position])
            self.set_flipper_pos_trajectory(t_now, [0, 3.0], pos)

        # if we are homing, need to set command differently
        if self.flipper_pos_traj is None and self.flipper_traj is None and self.chassis_traj is None:
            self.cmd.velocity = 0.0
        elif self.flipper_pos_traj is not None and t_now < self.flipper_pos_traj.end_time:
            t = min(t_now, self.flipper_pos_traj.end_time)
            [pos, vel, accel] = self.flipper_pos_traj.get_state(t)
            self.flipper_cmd.position = pos
            self.flipper_cmd.velocity = vel
            #nan = np.empty((1, 4))
            #nan.fill(np.nan)
            self.wheel_cmd.position = np.nan
            self.wheel_cmd.velocity = np.nan
        else:
            if self.chassis_traj is not None:
                # chassis update
                t = min(t_now, self.chassis_traj.end_time)
                [vel, acc, jerk] = self.chassis_traj.get_state(t)
                self.wheel_cmd.velocity = self.chassis_to_wheel_vel @ vel
                self.wheel_cmd.position += self.wheel_cmd.velocity * dt

            if self.flipper_traj is not None:
                # flipper update
                t = min(t_now, self.flipper_traj.end_time)
                [vel, acc, jerk] = self.flipper_traj.get_state(t)
                self.flipper_cmd.velocity = vel

                if self.joined_flipper_mode and self.flippers_aligned:
                    self.flipper_cmd.position = self.aligned_flipper_position

                self.flipper_cmd.position += vel * dt

        self.group.send_command(self.cmd)
        self.t_prev = t_now

    def set_flipper_pos_trajectory(self, t_now, times, positions):
        # when this is happening (homing) we don't use the other two trajectories
        self.chassis_traj = None
        self.flipper_traj = None
        times = np.array(times, dtype=np.float64)
        self.flipper_pos_traj = hebi.trajectory.create_trajectory(times + t_now, positions)

    def set_flipper_vel_trajectory(self, t_now, velocities):
        times = np.array([0, self.flipper_ramp_time], dtype=np.float64)
        vels = np.empty((4, 2))
        accs = np.empty((4, 2))
        jerks = np.empty((4, 2))

        if self.flipper_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            [vels[:, 0], accs[:, 0], jerks[:, 0]] = self.flipper_traj.get_state(t)
        else:
            vels[:, 0] = self.flipper_fbk.velocity
            accs[:, 0] = self.flipper_fbk.effort
            jerks[:, 0] = np.zeros(4)

        vels[:, 1] = velocities
        accs[:, 1] = np.zeros(4)
        jerks[:, 1] = np.zeros(4)

        self.flipper_traj = hebi.trajectory.create_trajectory(times, vels, accs, jerks)

    def set_chassis_vel_trajectory(self, t_now, velocities):
        times = np.array([0, self.chassis_ramp_time], dtype=np.float64)
        vels = np.empty((3, 2))
        accs = np.empty((3, 2))
        jerks = np.empty((3, 2))

        if self.chassis_traj is not None:
            t = min(t_now, self.chassis_traj.end_time)
            [vels[:, 0], accs[:, 0], jerks[:, 0]] = self.chassis_traj.get_state(t)
        else:
            vels[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.velocity
            accs[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.effort
            jerks[:, 0] = np.zeros(3)

        vels[:, 1] = velocities
        accs[:, 1] = np.zeros(3)
        jerks[:, 1] = np.zeros(3)

        self.chassis_traj = hebi.trajectory.create_trajectory(times, vels, accs, jerks)

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
    DISCONNECTED = auto()
    EXIT = auto()


class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, base, mobile_io):
        self.state = TreadyState(TreadyState.STARTUP)
        self.base = base
        self.mobile_io = mobile_io
        self.SPEED_MAX_LIN = 0.125 # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / base.WHEEL_BASE # rad/s

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

        if self.mobile_io.get_button_state(quit_btn) == 1:
            self.transition_to(TreadyState.EXIT)
        elif self.mobile_io.get_button_state(reset_pose_btn) == 1:
            self.transition_to(TreadyState.HOMING)

        # Flipper Control
        flip1 = self.mobile_io.get_axis_state(slider_flip1)
        flip2 = self.mobile_io.get_axis_state(slider_flip2)
        flip3 = self.mobile_io.get_axis_state(slider_flip3)
        flip4 = self.mobile_io.get_axis_state(slider_flip4)

        if self.mobile_io.get_button_state(joined_flipper_btn):
            self.base.joined_flipper_mode = True
            f_vel1 = max(abs(flip1), abs(flip2)) * np.sign(flip1 + flip2) * self.FLIPPER_VEL_SCALE
            f_vel2 = -f_vel1
            f_vel3 = max(abs(flip3), abs(flip4)) * np.sign(flip3 + flip4) * self.FLIPPER_VEL_SCALE
            f_vel4 = -f_vel3

        else:
            self.base.joined_flipper_mode = False
            f_vel1 = flip1 * self.FLIPPER_VEL_SCALE
            f_vel2 = -1 * flip2 * self.FLIPPER_VEL_SCALE
            f_vel3 = flip3 * self.FLIPPER_VEL_SCALE
            f_vel4 = -1 * flip4 * self.FLIPPER_VEL_SCALE

        flipper_vels = [f_vel1, f_vel2, f_vel3, f_vel4]
        base.set_flipper_vel_trajectory(t_now, flipper_vels)

        # Mobile Base Control
        joy_vel_fwd = self.mobile_io.get_axis_state(joy_fwd)
        joy_vel_rot = self.mobile_io.get_axis_state(joy_rot)

        vel_x = self.SPEED_MAX_LIN * joy_vel_fwd
        vel_y = 0
        vel_rot = self.SPEED_MAX_ROT * joy_vel_rot

        chassis_vels = [vel_x, vel_y, vel_rot]
        base.set_chassis_vel_trajectory(t_now, chassis_vels)

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
            self.transition_to(TreadyState.HOMING)

        elif self.state is TreadyState.HOMING:
            if base.trajectory_complete(t_now):
                self.transition_to(TreadyState.TELEOP)

        elif self.state is TreadyState.TELEOP:
            if not has_mobile_feedback:
                if t_now - self.mobile_last_fbk_t > 1.0:
                    print("mobileIO timeout, disabling motion")
                    self.transition_to(TreadyState.DISCONNECTED)
            else:
                self.process_mobile_input(t_now)

        elif self.state is TreadyState.DISCONNECTED:
            if has_mobile_feedback:
                self.mobile_last_fbk_t = t_now
                self.transition_to(TreadyState.TELEOP)

        elif self.state is TreadyState.EXIT:
            return False
        return True

    def transition_to(self, state):
        if state is TreadyState.HOMING:
            print("TRANSITIONING TO HOMING")
            base.set_color('magenta')
            m.clear_text()
            msg = 'Robot Homing Sequence\nPlease wait...'
            m.set_text(msg)

            # build trajectory
            flipper_positions = np.empty((4, 2))
            if self.state is TreadyState.STARTUP:
                flipper_positions[:, 0] = base.flipper_fbk.position
            else:
                flipper_positions[:, 0] = base.flipper_fbk.position_command
            flipper_positions[:, 1] = np.deg2rad([-15, 15, 15, -15])

            base.set_flipper_pos_trajectory(time(), [0.0, 5.0], flipper_positions)
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
            base.cancel_trajectory()
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
