from enum import Enum, auto
from dataclasses import dataclass, field
from time import time, sleep
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Sequence, Callable
    import numpy.typing as npt
    from hebi._internal.mobile_io import MobileIO
    from hebi.arm import Arm, Gripper


class ArmControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


@dataclass(kw_only=True)
class ArmJoystickInputs:
    delta_xyz: 'npt.NDArray[np.float32]' = field(default_factory=lambda: np.array([0., 0., 0.]))
    delta_rot: 'npt.NDArray[np.float64]' = field(default_factory=lambda: np.eye(3))
    gripper_closed: bool = False
    home: bool = False


class ArmJoystickControl:
    def __init__(self, arm: 'Arm', gripper: 'Gripper | None' = None, home_pose: 'Sequence[float] | npt.NDArray[np.float64] | None' = None, ik_seed: 'Sequence[float] | npt.NDArray[np.float64] | None' = None, homing_time: float = 5.0, traj_duration: float = 0.25, shoulder_flip_angle=-np.pi / 2, joint_limits=None):
        self.namespace = ''

        self.state = ArmControlState.STARTUP
        self.arm = arm
        self.gripper = gripper

        self.arm_xyz_home = [0.5, 0.0, 0.15]
        self.arm_rot_home: 'npt.NDArray[np.float64]' = R.from_euler('x', np.pi).as_matrix()
        self.homing_time = homing_time
        self.traj_duration = traj_duration

        # Set the IK seed for the arm
        self.arm_seed_ik = ik_seed if ik_seed is not None else np.array([0.01, 1.4, 1.8, 2.0, -1.57, 0.01])[:arm.size]
        assert len(self.arm_seed_ik) == arm.size, \
            f"IK seed must have the same size as the arm ({arm.size}), got {self.arm_seed_ik.shape[0]} elements."
        # Set the home position for the arm
        self.arm_home = home_pose if home_pose is not None else self.arm.ik_target_xyz_so3(
            self.arm_seed_ik,
            self.arm_xyz_home,
            self.arm_rot_home)

        self.homing_time = homing_time
        # TODO: GENERALIZE THIS
        self.shoulder_flip_angle = shoulder_flip_angle
        if joint_limits is not None:
            self.joint_limits = joint_limits
        else:
            self.joint_limits = np.empty((arm.size, 2))
            self.joint_limits[:, 0] = -np.inf
            self.joint_limits[:, 1] = np.inf

        self.xyz_curr = np.empty(3)
        self.rot_curr = np.empty((3, 3))
        self.joint_target = arm.last_feedback.position_command
        self.mobile_last_fbk_t = time()

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.arm.send()
        if self.gripper:
            self.gripper.send()

    def update(self, t_now: float, arm_input: 'ArmJoystickInputs | None' = None):
        self.arm.update()

        if self.state is self.state.EXIT:
            return

        if arm_input is None:
            if self.state is not self.state.DISCONNECTED and t_now - self.mobile_last_fbk_t > 1.0:
                print(self.namespace + "mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return

        # Reset the timeout
        self.mobile_last_fbk_t = t_now
        # Transition to teleop if mobileIO is reconnected
        if self.state is self.state.DISCONNECTED:
            self.mobile_last_fbk_t = t_now
            print(self.namespace + 'Controller reconnected, demo continued.')
            self.transition_to(t_now, self.state.TELEOP)

        # After startup, transition to homing
        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

        # If homing is complete, transition to teleop
        elif self.state is self.state.HOMING:
            if self.arm.at_goal:
                self.joint_target = self.arm.last_feedback.position_command
                self.transition_to(t_now, self.state.TELEOP)

        # Teleop mode
        elif self.state is self.state.TELEOP:
            if arm_input.home:
                self.transition_to(t_now, self.state.HOMING)
                return

            arm_goal = self.compute_arm_goal(arm_input)
            self.arm.set_goal(arm_goal)

            if self.gripper:
                gripper_closed = self.gripper.state == 1.0
                if arm_input.gripper_closed and not gripper_closed:
                    print('Gripper Close')
                    self.gripper.close()
                elif not arm_input.gripper_closed and gripper_closed:
                    print('Gripper Open')
                    self.gripper.open()

    def transition_to(self, t_now: float, state: ArmControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print(self.namespace + "TRANSITIONING TO HOMING")
            self.home(self.homing_time)

        elif state is self.state.TELEOP:
            print(self.namespace + "TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print(self.namespace + "mobileIO timeout, disabling motion")

        elif state is self.state.EXIT:
            print(self.namespace + "TRANSITIONING TO EXIT")

        self.state = state

    def compute_arm_goal(self, arm_inputs: ArmJoystickInputs):
        pos_curr = self.arm.last_feedback.position_command

        if np.any(np.isnan(pos_curr)):
            print('No position command, falling back to feedback position')
            pos_curr = self.arm.last_feedback.position

        try:
            self.arm.FK(pos_curr, xyz_out=self.xyz_curr, orientation_out=self.rot_curr)
        except ValueError:
            print(f'ERROR: Cannot compute FK with input position: {pos_curr}')
            exit()

        arm_xyz_target = self.xyz_curr + arm_inputs.delta_xyz

        r_x = R.from_euler('x', arm_inputs.delta_rot[0])
        r_y = R.from_euler('y', arm_inputs.delta_rot[1])
        r_z = R.from_euler('z', arm_inputs.delta_rot[2])
        wrist_rot = R.from_euler('z', -pos_curr[5])
        arm_rot_target = (R.from_matrix(self.rot_curr) * wrist_rot * r_x * r_y * r_z * wrist_rot.inv()).as_matrix()

        #curr_seed_ik = pos_curr
        #curr_seed_ik[2] = abs(curr_seed_ik[2])

        if self.joint_target[2] < self.shoulder_flip_angle:
            diff = abs(self.joint_target[2] - self.shoulder_flip_angle)
            self.joint_target[2] = self.shoulder_flip_angle + diff

        joint_target = self.arm.ik_target_xyz_so3(
            self.joint_target,
            arm_xyz_target,
            arm_rot_target)

        out_of_bounds = False
        for idx, joint in enumerate(joint_target):
            if joint < self.joint_limits[idx, 0] or joint > self.joint_limits[idx, 1]:
                out_of_bounds = True

        if not out_of_bounds:
            self.joint_target = joint_target

        arm_goal = hebi.arm.Goal(self.arm.size)
        arm_goal.add_waypoint(t=self.traj_duration, position=self.joint_target)
        return arm_goal

    def home(self, duration):
        g = hebi.arm.Goal(self.arm.size)
        g.add_waypoint(t=duration, position=self.arm_home)
        self.arm.set_goal(g)

    def stop(self):
        self.transition_to(time(), self.state.EXIT)


def setup_mobile_io(m: 'MobileIO'):
    m.resetUI()
    m.set_button_label(1, '⟲', blocking=False)
    m.set_button_label(2, '', blocking=False)
    m.set_button_label(3, 'Quit', blocking=False)
    m.set_button_label(4, '', blocking=False)
    m.set_button_label(5, 'grip', blocking=False)
    m.set_button_label(6, '\u2191', blocking=False)
    m.set_button_label(7, '', blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, '\u2193', blocking=False)

    m.set_axis_label(1, '', blocking=False)
    m.set_axis_label(2, 'rotate', blocking=False)
    m.set_axis_label(3, 'wrist', blocking=False)
    m.set_axis_label(4, 'XYZ\nScale', blocking=False)
    m.set_axis_label(5, 'Rot\nScale', blocking=False)
    m.set_axis_label(6, '', blocking=False)
    m.set_axis_label(7, '', blocking=False)
    m.set_axis_label(8, 'translate', blocking=False)

    m.set_snap(3, 0)
    m.set_snap(4, np.nan)
    m.set_snap(5, np.nan)
    m.set_axis_value(4, 0.0)
    m.set_axis_value(5, 0.0)

def parse_mobile_io_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return False, None

    if m.get_button_state(3):
        return True, None

    if m.get_button_state(1):
        return False, ArmJoystickInputs(home=True)
    
    xyz_scale = (m.get_axis_state(4) + 1.0) / 30.0
    rot_scale = (m.get_axis_state(5) + 1.0) / 10.0

    arm_dx = xyz_scale * m.get_axis_state(8)
    arm_dy = -xyz_scale * m.get_axis_state(7)

    arm_dz = 0.0
    if m.get_button_state(6):
        arm_dz = xyz_scale
    elif m.get_button_state(8):
        arm_dz = -xyz_scale

    arm_drx = -rot_scale * m.get_axis_state(2)
    arm_dry = -rot_scale * m.get_axis_state(1)
    arm_drz = -rot_scale * m.get_axis_state(3)

    arm_input = ArmJoystickInputs(
        delta_xyz=[arm_dx, arm_dy, arm_dz],
        delta_rot=[arm_drx, arm_dry, arm_drz],
        gripper_closed=m.get_button_state(5),
    )

    return False, arm_input


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Control a HEBI arm with joystick input')
    parser.add_argument('--enable-logging', action='store_true', help='Enable logging')
    args = parser.parse_args()

    lookup = hebi.Lookup()
    sleep(2)

    # Arm setup
    root_dir = os.path.abspath(os.path.dirname(__file__))
    cfg_file = os.path.join(root_dir, "config", "A-2580-06G.cfg.yaml")
    cfg = hebi.config.load_config(cfg_file)
    family = cfg.families[0]

    # Create Arm object
    arm = hebi.arm.create_from_config(cfg, lookup)

    enable_logging = args.enable_logging
    if enable_logging:
        arm.group.start_log('dir', 'logs', mkdirs=True)

    # Check if the gripper is disabled
    use_gripper = cfg.user_data.get('has_gripper', False)
    gripper = None
    if use_gripper:
        gripper_family = family
        gripper_name = 'gripperSpool'

        gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])
        while gripper_group is None:
            print(f"Looking for gripper module {gripper_family} / {gripper_name} ...")
            sleep(1)
            gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])

        print("Gripper module found.")
        gripper = hebi.arm.Gripper(gripper_group, -5, 1)
        gripper_gains = os.path.join(root_dir, "config/gains/gripper_spool_gains.xml")
        gripper.load_gains(gripper_gains)

    joint_limits = np.empty((7, 2))
    joint_limits[:, 0] = -np.inf
    joint_limits[:, 1] = np.inf

    # base limits [-2, 2] (radians)
    # joint_limits[0, :] = [-2.0, 2.0]
    # shoulder limits [-2, inf]
    # joint_limits[1, 0] = -2.0

    arm_control = ArmJoystickControl(arm,
                                     gripper,
                                     ik_seed=cfg.user_data.get('ik_seed_pos', None),
                                     joint_limits=joint_limits
                                     )

    # Setup MobileIO
    print('Looking for mobileIO device...')
    m = create_mobile_io(lookup, family)
    while m is None:
        try:
            print('Waiting for mobileIO device to come online...')
            sleep(1)
            m = create_mobile_io(lookup, family)
        except KeyboardInterrupt as e:
            exit()

    print("mobileIO device found.")
    m.update()
    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    while arm_control.running:
        t = time()
        try:
            quit, arm_input = parse_mobile_io_feedback(m)
            if quit:
                break
            arm_control.update(t, arm_input)
            arm_control.send()
        except KeyboardInterrupt as e:
            break

    arm_control.stop()
    if enable_logging:
        hebi_log = arm.group.stop_log()
