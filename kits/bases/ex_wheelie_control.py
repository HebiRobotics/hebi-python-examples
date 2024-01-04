#!/usr/bin/env python3

import hebi
import numpy as np
from time import sleep, time
import hebi
from hebi.util import create_mobile_io


class WheelieDrive:
    WHEEL_RADIUS = 0.10  # m
    BASE_RADIUS = 0.43  # m (half of distance between diff drive wheel centers)

    def __init__(self, group):
        self.group = group
        self.base_command = hebi.GroupCommand(group.size)
        self.base_feedback = hebi.GroupFeedback(group.size)
        self.trajectory = None
        self.trajectory_start_time = 0
        self.start_wheel_pos = np.zeros(self.group.size)
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

        cmd_vels = np.zeros(self.group.size)
        if self.trajectory is None:
            self.start_wheel_pos = self.base_feedback.position
        else:
            t = min(t_now - self.trajectory_start_time, self.trajectory.duration)
            cmd_pos, cmd_vels, _ = self.trajectory.get_state(t)
            self.start_wheel_pos += cmd_pos

        target_vel_wheels = np.zeros(self.group.size)
        target_vel_wheels[0] = -1 * dtheta * (self.BASE_RADIUS / self.WHEEL_RADIUS)
        target_vel_wheels[1] = -1 * dtheta * (self.BASE_RADIUS / self.WHEEL_RADIUS)

        target_vel_wheels[0] += dx / self.WHEEL_RADIUS
        target_vel_wheels[1] -= dx / self.WHEEL_RADIUS

        target_vel_wheels[2] = target_vel_wheels[0]
        target_vel_wheels[3] = target_vel_wheels[1]

        velocities = np.empty((self.group.size, 4))
        velocities[:, 0] = cmd_vels
        velocities[:, 1] = target_vel_wheels
        velocities[:, 2] = target_vel_wheels
        velocities[:, 3] = 0.0

        self.build_velocity_trajectory(t_now, times, velocities)

    def build_velocity_trajectory(self, t_now, times, velocities):

        # start position 0, unconstrained waypoints afterwards
        p = np.empty((self.group.size, 4))
        p.fill(np.nan)
        p[:, 0] = 0

        a = np.zeros((self.group.size, 4))

        self.trajectory = hebi.trajectory.create_trajectory(times, p, velocities, a)
        self.trajectory_start_time = t_now


if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    # Base setup
    base_family = "Chevron"
    module_names = ['W1_front-left', 'W2_front-right', 'W3_rear-left', 'W4_rear-right']

    # Create group
    group = lookup.get_group_from_names([base_family], module_names)
    if group is None:
        raise RuntimeError(f"Could not find Wheelie modules: {module_names} in family '{base_family}'")

    base = DiffDrive(group)

    print('Waiting for Mobile IO device to come online...')
    m = create_mobile_io(lookup, base_family, phone_name)
    while m is None:
        print('Looking for MobileIO...')
        time.sleep(1)
        m = create_mobile_io(lookup, base_family)
    m.set_led_color("blue")
    m.update()

    # Demo Variables
    abort_flag = False

    # Print Instructions
    instructions = "B8 - Quit"

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
