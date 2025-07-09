#!/usr/bin/env python3

import hebi
import numpy as np
from time import sleep, time
import hebi
from hebi.util import create_mobile_io


class ChassisVelocity:
    def __init__(self, x: float = 0, y: float = 0, rz: float = 0):
        self.x = x
        self.y = y
        self.rz = rz
    
    def __repr__(self) -> str:
        return f'ChassisVelocity(x={self.x}, y={self.y}, rz={self.rz})'


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


class OmniBase:
    WHEEL_RADIUS = 0.0762 # m
    BASE_RADIUS = 0.220   # m (center of omni to origin of base)

    def __init__(self, group):
        self.group = group
        self.base_command = hebi.GroupCommand(group.size)
        self.base_feedback = hebi.GroupFeedback(group.size)
        self.trajectory = None
        self.color = hebi.Color(0, 0, 0)

        self.vels_base_to_wheel = self._build_jacobian(self.BASE_RADIUS, self.WHEEL_RADIUS)

    def _build_jacobian(self, base_radius, wheel_radius):
        vels_base_to_wheel = np.zeros((3, 3))
        vels_base_to_wheel[0, 0] = -np.sqrt(3) / 2.0
        vels_base_to_wheel[1, 0] = np.sqrt(3) / 2.0
        vels_base_to_wheel[2, 0] = 0.0

        vels_base_to_wheel[0, 1] = -0.5
        vels_base_to_wheel[1, 1] = -0.5
        vels_base_to_wheel[2, 1] = 1.0

        vels_base_to_wheel[:, 2] = -base_radius
        vels_base_to_wheel /= wheel_radius
        return vels_base_to_wheel


    def update(self, t_now):
        if not self.group.get_next_feedback(reuse_fbk=self.base_feedback):
            return False

        if self.trajectory:
            p, v, _ = self.trajectory.get_state(t_now)
            theta = p[2]

            world_to_local_rot = np.zeros((3, 3))
            world_to_local_rot[0, 0] = np.cos(theta)
            world_to_local_rot[1, 1] = np.cos(theta)
            world_to_local_rot[0, 1] = -np.sin(theta)
            world_to_local_rot[1, 0] = np.sin(theta)
            world_to_local_rot[2, 2] = 1.0

            v_local = world_to_local_rot @ v
            #self.base_command.position = self.start_wheel_pos + p
            self.base_command.velocity = self.vels_base_to_wheel @ v_local
            self.base_command.led.color = self.color
            #print(f"P: {p}")
            #print(f"V: {v}")
    
    def send(self):
        self.group.send_command(self.base_command)

    def build_smooth_velocity_trajectory(self, dx, dy, dtheta, t_now):
        times = np.array([0, 0.15, 0.9, 1.2])

        cmd_vels = np.zeros(3)
        if self.trajectory is not None:
            _, cmd_vels, _ = self.trajectory.get_state(t_now)

        target_vel = np.array([dx, dy, dtheta])

        velocities = np.empty((3, 4))
        velocities[:, 0] = cmd_vels
        velocities[:, 1] = target_vel
        velocities[:, 2] = target_vel
        velocities[:, 3] = 0.0

        self.build_velocity_trajectory(t_now + times, velocities)

    def build_velocity_trajectory(self, times, velocities):

        # start position 0, unconstrained waypoints afterwards
        p = np.full((3, 4), np.nan)
        p[:, 0] = 0

        a = np.zeros((3, 4))

        self.trajectory = hebi.trajectory.create_trajectory(times, p, velocities, a)


if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    # Base setup
    base_family = "HEBI"
    module_names = ['W1', 'W2', 'W3']

    # Create group
    group = lookup.get_group_from_names([base_family], module_names)
    if group is None:
        raise RuntimeError(f"Could not find OmniBase modules: {module_names} in family '{base_family}'")
    load_gains(group, "gains/omni_base.xml")

    base = OmniBase(group)

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
        dy = pow(m.get_axis_state(7), 3)

        base.build_smooth_velocity_trajectory(dx, dy, dtheta, time())
