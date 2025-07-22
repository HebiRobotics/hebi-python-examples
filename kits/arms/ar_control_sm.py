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
class ArmMobileIOInputs:
    phone_pos: 'npt.NDArray[np.float32]' = field(default_factory=lambda: np.array([0., 0., 0.]))
    phone_rot: 'npt.NDArray[np.float64]' = field(default_factory=lambda: np.eye(3))
    xyz_scaling: float = 1.0
    lock_toggle: bool = False
    locked: bool = True
    gripper_closed: bool = False
    home: bool = False


class ArmMobileIOControl:
    def __init__(self, arm: 'Arm', gripper: 'Gripper | None' = None, home_pose: 'Sequence[float] | npt.NDArray[np.float64] | None' = None, ik_seed: 'Sequence[float] | npt.NDArray[np.float64] | None' = None, homing_time: float = 5.0, traj_duration: float = 0.25, xyz_scale: 'npt.NDArray[np.float64]' = np.ones(3)):
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

        self.xyz_scale = xyz_scale

        self.last_locked_xyz = self.arm_xyz_home.copy()
        self.last_locked_rot = self.arm_rot_home.copy()

        self.locked = True
        self._transition_handlers: 'list[Callable[[ArmMobileIOControl, ArmControlState], None]]' = [
        ]
        self.mobile_last_fbk_t = time()

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.arm.send()
        if self.gripper:
            self.gripper.send()

    def update(self, t_now: float, arm_input: 'ArmMobileIOInputs | None' = None):
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
            # Lock arm when reconnected
            self.locked = True
            self.transition_to(t_now, self.state.TELEOP)

        # After startup, transition to homing
        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

        # If homing is complete, transition to teleop
        elif self.state is self.state.HOMING:
            if self.arm.at_goal:
                self.phone_xyz_init = arm_input.phone_pos.copy()
                self.phone_rot_init = arm_input.phone_rot.copy()

                xyz = np.empty(3)
                orientation = np.empty((3, 3))
                self.arm.FK(self.arm.last_feedback.position,
                            xyz_out=xyz,
                            orientation_out=orientation)

                self.last_locked_xyz = xyz.copy()
                self.last_locked_rot = orientation.copy()
                self.locked = True  # Lock the arm after reaching home

                self.transition_to(t_now, self.state.TELEOP)

        # Teleop mode
        elif self.state is self.state.TELEOP:
            if arm_input.home:
                self.transition_to(t_now, self.state.HOMING)
                return

            if arm_input.lock_toggle:
                self.locked = arm_input.locked
                if not self.locked:
                    self.phone_xyz_init = arm_input.phone_pos.copy()
                    self.phone_rot_init = arm_input.phone_rot.copy()

                    xyz = np.zeros(3)
                    orientation = np.zeros((3, 3))
                    self.arm.FK(self.arm.last_feedback.position,
                                xyz_out=xyz,
                                orientation_out=orientation)

                    self.last_locked_xyz = xyz.copy()
                    self.last_locked_rot = orientation.copy()

            if not self.locked:
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

        for handler in self._transition_handlers:
            handler(self, state)
        self.state = state

    def compute_arm_goal(self, arm_input: ArmMobileIOInputs):
        phone_offset = arm_input.phone_pos - self.phone_xyz_init
        arm_xyz_target = self.last_locked_xyz + arm_input.xyz_scaling * \
            self.xyz_scale * (self.phone_rot_init.T @ phone_offset)
        arm_rot_target = (arm_input.phone_rot @ self.phone_rot_init.T) @ self.last_locked_rot

        # if ar scaling is 0, move the home AR pose to current pose
        # this keeps the arm from driving to some weird offset when scaling
        # is turned back up by the user in the future
        if arm_input.xyz_scaling == 0.0:
            self.phone_xyz_home = arm_input.phone_pos

        joint_target = self.arm.ik_target_xyz_so3(
            self.arm.last_feedback.position,
            arm_xyz_target,
            arm_rot_target)

        arm_goal = hebi.arm.Goal(self.arm.size)
        arm_goal.add_waypoint(t=self.traj_duration, position=joint_target)
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
    m.set_button_label(3, '', blocking=False)
    m.set_button_label(4, '', blocking=False)
    m.set_button_label(5, 'lock', blocking=False)
    m.set_button_mode(5, 1)
    m.set_button_label(6, '', blocking=False)
    m.set_button_label(7, 'grip', blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, '❌', blocking=False)

    m.set_axis_label(1, '', blocking=False)
    m.set_axis_label(2, '', blocking=False)
    m.set_axis_label(3, '', blocking=False)
    m.set_axis_label(4, 'XYZ\nScale', blocking=False)
    m.set_axis_label(5, '', blocking=False)
    m.set_axis_label(6, '', blocking=False)
    m.set_axis_label(7, '', blocking=False)
    m.set_axis_label(8, '', blocking=False)

    m.set_snap(4, np.nan)
    m.set_axis_value(4, 1.0)

def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return False, None

    if m.get_button_state(8):
        return True, None

    if m.get_button_state(1):
        return False, ArmMobileIOInputs(home=True)

    try:
        wxyz = m.orientation
        xyzw = [*wxyz[1:], wxyz[0]]
        rotation = R.from_quat(xyzw).as_matrix()
    except ValueError as e:
        print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
        rotation = np.eye(3)

    arm_input = ArmMobileIOInputs(
        phone_pos=np.copy(m.position),
        phone_rot=rotation,
        lock_toggle=(m.get_button_diff(5) != 0),
        locked=(not m.get_button_state(5)),
        gripper_closed=m.get_button_state(7),
        xyz_scaling=(m.get_axis_state(4) + 1.0) / 2.0
    )

    return False, arm_input


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Control a HEBI arm with AR input')
    parser.add_argument('--enable-logging', action='store_true', help='Enable logging')
    args = parser.parse_args()

    lookup = hebi.Lookup()
    sleep(2)

    # Arm setup
    root_dir = os.path.abspath(os.path.dirname(__file__))
    cfg_file_path = os.path.join(root_dir, "config/A-2580-06G.cfg.yaml")
    cfg = hebi.config.load_config(cfg_file_path)
    arm_family = cfg.families[0]

    # Create Arm object
    arm = hebi.arm.create_from_config(cfg, lookup)

    enable_logging = args.enable_logging
    if enable_logging:
        arm.group.start_log('dir', 'logs', mkdirs=True)

    # Check if the gripper is disabled
    use_gripper = cfg.user_data.get('has_gripper', False)
    gripper = None
    if use_gripper:
        gripper_family = arm_family
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

    arm_control = ArmMobileIOControl(arm,
                                     gripper,
                                     ik_seed=cfg.user_data.get('ik_seed_pos', None)
                                     )

    # Setup MobileIO
    print('Looking for mobileIO device...')
    m = create_mobile_io(lookup, arm_family)
    while m is None:
        try:
            print('Waiting for mobileIO device to come online...')
            sleep(1)
            m = create_mobile_io(lookup, arm_family)
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
            quit, arm_input = parse_mobile_feedback(m)
            if quit:
                break
            arm_control.update(t, arm_input)
            arm_control.send()
        except KeyboardInterrupt as e:
            break

    arm_control.stop()
    if enable_logging:
        hebi_log = arm.group.stop_log()
