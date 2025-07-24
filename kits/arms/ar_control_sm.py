
from enum import Enum, auto
from dataclasses import dataclass, field
from time import time, sleep
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Callable
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
    ar_scaling: float = 1.0
    lock_toggle: bool = False
    locked: bool = True
    gripper_closed: bool = False
    home: bool = False


class ArmMobileIOControl:
    def __init__(self, arm: 'Arm', gripper: 'Gripper | None' = None, homing_time=5.0, traj_duration=1.0, xyz_scale: 'npt.NDArray[np.float64]' = np.ones(3)):
        self.namespace = ''

        self.state = ArmControlState.STARTUP
        self.arm = arm
        self.gripper = gripper

        # 5 DoF home
        # self.arm_xyz_home = [0.34, 0.0, 0.23]
        # 6 DoF home
        self.arm_xyz_home = [0.5, 0.0, 0.0]
        self.arm_rot_home: 'npt.NDArray[np.float64]' = (R.from_euler(
            'z', np.pi / 2) * R.from_euler('x', np.pi)).as_matrix()
        self.homing_time = homing_time
        self.traj_duration = traj_duration

        # 5 DoF seed
        # self.arm_seed_ik = np.array([0.25, -1.0, 0, -0.75, 0])
        # 6 DoF seed
        self.arm_seed_ik = np.array([0.3, 1.2, 2.2, 2.9, -1.57, 0])
        self.arm_home = self.arm.ik_target_xyz_so3(
            self.arm_seed_ik,
            self.arm_xyz_home,
            self.arm_rot_home)

        self.xyz_scale = xyz_scale

        self.last_locked_xyz = self.arm_xyz_home.copy()
        self.last_locked_rot = self.arm_rot_home.copy()
        self.last_locked_seed = self.arm_home.copy()

        self.locked = True
        self._transition_handlers: 'list[Callable[[ArmMobileIOControl, ArmControlState], None]]' = [
        ]
        self.last_cmd_t = time()

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.arm.send()
        if self.gripper:
            self.gripper.send()

    def update(self, t_now: float, arm_input: 'ArmMobileIOInputs | None'):
        self.arm.update()

        if self.state is self.state.EXIT:
            return

        if arm_input is None:
            if self.state is not self.state.DISCONNECTED and t_now - self.last_cmd_t > 1.0:
                print(self.namespace + "mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return

        # Reset the timeout
        self.last_cmd_t = t_now
        # Transition to teleop if mobileIO is reconnected
        last_pos = self.arm.last_feedback.position
        if self.state is self.state.DISCONNECTED:
            self.last_cmd_t = t_now
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
                self.phone_xyz_home = arm_input.phone_pos
                self.phone_rot_home = arm_input.phone_rot

                self.last_locked_seed = last_pos.copy()
                xyz = np.empty(3)
                orientation = np.empty((3, 3))
                self.arm.FK(last_pos,
                            xyz_out=xyz,
                            orientation_out=orientation)

                self.last_locked_xyz = xyz
                self.last_locked_rot = orientation

                self.transition_to(t_now, self.state.TELEOP)

        # Teleop mode
        elif self.state is self.state.TELEOP:
            if arm_input.home:
                self.transition_to(t_now, self.state.HOMING)
                return

            if arm_input.lock_toggle:
                self.locked = arm_input.locked

            if not self.locked:
                arm_goal = self.compute_arm_goal(arm_input)
                if arm_goal is not None:
                    self.arm.set_goal(arm_goal)
            else:
                self.phone_xyz_home = arm_input.phone_pos
                self.phone_rot_home = arm_input.phone_rot

                self.last_locked_seed = last_pos.copy()
                xyz = np.zeros(3)
                orientation = np.zeros((3, 3))
                self.arm.FK(last_pos,
                            xyz_out=xyz,
                            orientation_out=orientation)

                self.last_locked_xyz = xyz
                self.last_locked_rot = orientation

            if self.gripper is not None:
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
        phone_offset = arm_input.phone_pos - self.phone_xyz_home
        rot_mat = self.phone_rot_home
        arm_xyz_target = self.last_locked_xyz + arm_input.ar_scaling * \
            self.xyz_scale * (rot_mat.T @ phone_offset)
        arm_rot_target = rot_mat.T @ arm_input.phone_rot @ self.last_locked_rot

        # if ar scaling is 0, move the home AR pose to current pose
        # this keeps the arm from driving to some weird offset when scaling
        # is turned back up by the user in the future
        if arm_input.ar_scaling == 0.0:
            self.phone_xyz_home = arm_input.phone_pos

        joint_target = self.arm.ik_target_xyz_so3(
            self.last_locked_seed,
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
    m.set_button_label(1, '⟲')
    m.set_button_label(5, 'lock')
    m.set_button_mode(5, 1)
    m.set_button_label(7, 'grip')
    m.set_button_mode(7, 1)
    m.set_button_label(8, '❌')


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
        lock_toggle=m.get_button_state(5),
        locked=m.get_button_state(7))

    return False, arm_input


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Arm setup
    cfg = hebi.config.load_config('./config/A-2580-06.cfg.yaml')
    arm = hebi.arm.create_from_config(cfg, lookup)

    arm_control = ArmMobileIOControl(arm)

    arm_family = cfg.families[0]

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
    m.resetUI()
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
