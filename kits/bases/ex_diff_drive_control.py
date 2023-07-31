#!/usr/bin/env python3

import hebi
import numpy as np
from time import sleep, time
import hebi
from hebi.util import create_mobile_io


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


class DiffDrive:
    WHEEL_RADIUS = 0.10  # m
    BASE_RADIUS = 0.43  # m (half of distance between diff drive wheel centers)

    def __init__(self, group):
        self.group = group
        self.base_command = hebi.GroupCommand(group.size)
        self.base_feedback = hebi.GroupFeedback(group.size)
        self.trajectory = None
        self.trajectory_start_time = 0
        self.start_wheel_pos = np.array([0, 0])
        self.color = hebi.Color(0, 0, 0)

    def update(self, t_now):
        if not self.group.get_next_feedback(reuse_fbk=self.base_feedback):
            return False

        if self.trajectory:
            t = min(t_now - self.trajectory_start_time, self.trajectory.duration)
            p, v, _ = self.trajectory.get_state(t)
            self.base_command.position = self.start_wheel_pos + p
            self.base_command.velocity = v

            self.base_command.led.color = self.color
            #print(f"P: {p}")
            print(f"V: {v}")
            self.group.send_command(self.base_command)

    def build_smooth_velocity_trajectory(self, dx, dtheta, t_now):
        times = np.array([0, 0.15, 0.9, 1.2])

        cmd_vels = np.zeros(2)
        if self.trajectory is None:
            self.start_wheel_pos = self.base_feedback.position
        else:
            t = min(t_now - self.trajectory_start_time, self.trajectory.duration)
            cmd_pos, cmd_vels, _ = self.trajectory.get_state(t)
            self.start_wheel_pos += cmd_pos

        target_vel_wheels = np.zeros(2)
        target_vel_wheels[0] = -1 * dtheta * (self.BASE_RADIUS / self.WHEEL_RADIUS)
        target_vel_wheels[1] = -1 * dtheta * (self.BASE_RADIUS / self.WHEEL_RADIUS)

        target_vel_wheels[0] += dx / self.WHEEL_RADIUS
        target_vel_wheels[1] -= dx / self.WHEEL_RADIUS

        velocities = np.empty((2, 4))
        velocities[:, 0] = cmd_vels
        velocities[:, 1] = target_vel_wheels
        velocities[:, 2] = target_vel_wheels
        velocities[:, 3] = [0, 0]

        self.build_velocity_trajectory(t_now, times, velocities)

    def build_velocity_trajectory(self, t_now, times, velocities):

        # start position 0, unconstrained waypoints afterwards
        p = np.empty((2, 4))
        p.fill(np.nan)
        p[:, 0] = 0

        a = np.zeros((2, 4))

        self.trajectory = hebi.trajectory.create_trajectory(times, p, velocities, a)
        self.trajectory_start_time = t_now


if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    # Base setup
    base_family = "HEBI"
    module_names = ['W1_left', 'W2_right']

    # Create group
    group = lookup.get_group_from_names([base_family], module_names)
    if group is None:
        raise RuntimeError(f"Could not find Diff Drive modules: {module_names} in family '{base_family}'")
    load_gains(group, "gains/diff_drive.xml")

    base = DiffDrive(group)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for Mobile IO device to come online...')
    m = create_mobile_io(lookup, base_family, phone_name)
    if m is None:
        raise RuntimeError("Could not find Mobile IO device")
    m.set_led_color("blue")
    m.update()

    # Demo Variables
    abort_flag = False

    # Print Instructions
    instructions = "B8 - Quit"
    print(instructions)
    m.clear_text()
    m.add_text(instructions)

    #######################
    ## Main Control Loop ##
    #######################

    while not abort_flag:
        base.update(time())

        if not m.update():
            print("Failed to get feedback from MobileIO")
            continue

        # B8 - Quit
        if m.get_button_diff(8) == 1:  # "ToOn"
            # Reset text & color, and quit
            m.clear_text()
            m.set_led_color("transparent")
            abort_flag = True
            break

        dtheta = pow(m.get_axis_state(1), 3) * 2.0
        dx = pow(m.get_axis_state(8), 3)

        base.build_smooth_velocity_trajectory(dx, dtheta, time())
