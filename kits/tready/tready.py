import hebi
import sys
from time import time
from enum import Enum, auto


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
        self.flipper_fbk = fbk.create_view([1, 2, 3, 4])
        self.wheel_fbk = fbk.create_view([5, 6, 7, 8])
        self.cmd = hebi.GroupFeedback(group.size)
        self.flipper_cmd = cmd.create_view([1, 2, 3, 4])
        self.wheel_cmd = cmd.create_view([5, 6, 7, 8])

        self.joined_flipper_mode = False

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.group.get_next_feedback(reuse_fbk=self.fbk):
        t_now = time()

        times = np.array([0.0, chassis_ramp_time]) += t_now
        self.chassis_traj = hebi.trajectory.create_trajectory(times, np.zeros((2, 3)))
        times = np.array([0.0, flipper_ramp_time]) += t_now
        self.flipper_traj = hebi.trajectory.create_trajectory(times, np.zeros((2, 4)))
        self.flipper_pos_traj = None

        self.cmd.position = self.fbk.position
        self.t_prev = t_now

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
        dt = t_now - t_prev

        # switch to position trajectory to align flippers if needed
        if self.joined_flipper_mode and !self.flippers_aligned:
            # this sets flipper_pos_traj, so flippers align
            pos = np.array([self.flipper_cmd.position, self.aligned_flipper_position])
            self.set_flipper_pos_trajectory(t_now, [0, 3.0], pos)

        # if we are homing, need to set command differently
        if self.flipper_pos_traj is not None:
            t = min(t_now, self.flipper_pos_traj.end_time)
            [pos, vel, accel] = self.flipper_pos_traj.get_state(t)
            self.flipper_cmd.position = pos
            self.flipper_cmd.velocity = vel
            nan = np.empty((1, 4))
            nan.fill(np.nan)
            self.wheel_cmd.position = nan
            self.wheel_cmd.velocity = nan
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
        self.flipper_pos_traj = hebi.trajectory.create_trajectory(times += t_now, positions)

    def set_flipper_vel_trajectory(self, t_now, velocities):
        if self.flipper_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            [vel, acc, jerk] = self.flipper_traj.get_state(t)
        else:
            vel = self.flipper_fbk.velocity
            acc = self.flipper_fbk.effort
            jerk = np.zeros((1, 4))

        vels = np.array([vel, velocities])
        accs = [acc, np.zeros((1, 4))]
        jerks = [jerk; np.zeros((1, 4))]
        times = np.array([0, self.flipper_ramp_time], dtype=np.float64)
        self.flipper_traj = hebi.trajectory.create_trajectory(times, vels, accs, jerks)

    def set_chassis_vel_trajectory(self, t_now, velocities):
        if self.chassis_traj is not None:
            t = min(t_now, self.chassis_traj.end_time)
            [vel, acc, jerk] = self.chassis_traj.get_state(t)
        else:
            vel = self.wheel_to_chassis_vel * self.wheel_fbk.velocity
            acc = self.wheel_to_chassis_vel * self.wheel_fbk.effort
            jerk = np.zeros((1, 3))

        vels = np.array([vel, velocities])
        accs = [acc, np.zeros((1, 3))]
        jerks = [jerk; np.zeros((1, 3))]
        times = np.array([0, self.chassis_ramp_time], dtype=np.float64)
        self.chassis_traj = hebi.trajectory.create_trajectory(times, vels, accs, jerks)


class TreadyControl:
    class TreadyState(Enum):
        STARTUP = auto()
        HOMING = auto()
        TELEOP = auto()
        DISCONNECTED = auto()
        EXIT = auto()

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, base, mobile_io):
        self.state = TreadyState(TreadyState.STARTUP)
        self.base = base
        self.mobile_io = mobile_io

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

        base.set_flipper_vel_trajectory(t_now, [f_vel1, f_vel2, f_vel3, f_vel4])

        # Mobile Base Control
        joy_vel_fwd = self.mobile_io.get_axis_state(joy_fwd)
        joy_vel_rot = self.mobile_io.get_axis_state(joy_rot)

        vel_x = speed_max_lin * joy_vel_fwd
        vel_y = 0
        vel_rot = speed_max_rot * joy_vel_rot

        base.set_chassis_vel_trajectory(t_now, [vel_x, vel_y, vel_rot])

    def update(self, t_now):
        has_mobile_feedback = self.mobile_io.update()
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
            if base.trajectory_complete():
                self.transition_to(TreadyState.TELEOP)

        elif self.state is TreadyState.TELEOP:
            if not has_mobile_feedback:
                if t_now - self.mobile_last_fbk > 1.0:
                    print("mobileIO timeout, disabling motion")
                    self.transition_to(TreadyState.DISCONNECTED)

            self.mobile_last_fbk = t_now
            self.process_mobile_input()

        elif self.state is TreadyState.DISCONNECTED:
            if has_mobile_feedback:
                self.mobile_last_fbk = t_now
                self.transition_to(TreadyState.TELEOP)

        elif self.state is TreadyState.EXIT:
            return false
        return true

    def transition_to(self, state):
        if state is TreadyState.HOMING:
            base.set_color('magenta')
            m.clear_text()
            msg = '''
            Robot Homing Sequence
            Please wait...
            '''
            m.set_text(msg)

            # build trajectory
            flipper_positions = np.empty((2, 4))
            if self.state is TreadyState.STARTUP:
                flipper_positions[0, :] = base.flipper_feedback.position
            else:
                flipper_positions[0, :] = base.flipper_feedback.position_command
            flipper_positions[1, :] = np.deg2rad([-15, 15, 15, -15])

            base.set_flipper_pos_trajectory(time(), [0.0 5.0], flipper_positions)
        elif state is TreadyState.TELEOP:
            base.clear_color()
            # Print Instructions
            m.clear_text()
            instructions = '''
            Robot Ready to Control
            B1: Reset
            B6: Joined Flipper
            B8 - Quit
            '''
            m.set_text(instructions)
            m.set_led_color("green")
        elif state is TreadyState.DISCONNECTED:
            base.cancel_trajectory()
            base.set_color('blue')
        elif state is TreadyState.EXIT:
            # unset mobileIO control config
            self.mobile_io.set_led_color("red")
            self.base.set_color("red")

            self.mobile_io.set_button_mode(6, 0)
            self.mobile_io.set_button_output(1, 0)
            self.mobile_io.set_button_output(8, 0)
            m.clear_text()
            m.set_text('Demo Stopped.')


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
        raise RuntimeError(f"Could not find modules: {module_names} in family '{base_family}'")
    load_gains(group, "gains/tready.xml")

    base = TreadedBase(group)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for mobileIO device to come online...')
    m = create_mobile_io(lookup, base_family, phone_name)
    while m is None:
        m = create_mobile_io(lookup, base_family, phone_name)
        print("Could not find mobileIO device, waiting...")
        time.sleep(0.5)

    print("mobileIO device found.")

    #######################
    ## Main Control Loop ##
    #######################
    demo_controller = TreadyControl(base, m)

    while not demo_controller.update(time()):
        pass
    sys.exit(0)
