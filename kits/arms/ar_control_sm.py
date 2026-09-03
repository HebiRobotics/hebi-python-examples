
from enum import Enum, auto
from dataclasses import dataclass
from time import time, sleep
import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Callable
    from numpy.typing import NDArray
    from hebi._internal.mobile_io import MobileIO
    from hebi.arm import Arm, Gripper


class ArmControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    IDLE = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


@dataclass(kw_only=True)
class ArmMobileIOInputs:
    phone_pos: 'NDArray[np.float32]'
    phone_rot: 'NDArray[np.float64]'
    ar_scaling: float = 1.0
    lock_toggle: bool = False
    locked: bool = True
    gripper_closed: bool = False
    home: bool = False


class ArmMobileIOControl:
    def __init__(self,
                 arm: 'Arm',
                 gripper: 'Gripper | None' = None,
                 homing_time=5.0,
                 traj_duration=1.0,
                 xyz_scale: 'NDArray[np.float64]' = np.ones(3),
                 ):

        self.namespace = ''

        self.state = ArmControlState.STARTUP
        self.arm = arm
        self.gripper = gripper

        self.homing_time = homing_time
        self.traj_duration = traj_duration
        self.xyz_scale = xyz_scale

        self.arm_ik_seed = np.array([0.3, 1.2, 2.2, 2.9, -1.57, 0])

        home_orientation = R.from_euler('z', np.pi / 2) * R.from_euler('x', np.pi)
        self.set_arm_home(
            np.array([0.5, 0.0, 0.0]),
            home_orientation.as_matrix())

        self._transition_handlers: 'list[Callable[[ArmMobileIOControl, ArmControlState], None]]' = [
        ]
        self.last_cmd_t = time()

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def set_arm_home(self, xyz, rot):
        self.arm_xyz_home = xyz
        self.arm_rot_home = rot
        self.arm_home = self.arm.ik_target_xyz_so3(
            self.arm_ik_seed,
            xyz, rot)

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
        last_pos = self.arm.last_feedback.position_command
        if np.any(np.isnan(last_pos)):
            last_pos = self.arm.last_feedback.position
        if self.state is self.state.DISCONNECTED:
            self.last_cmd_t = t_now
            print(self.namespace + 'Controller reconnected, demo continued.')
            self.transition_to(t_now, self.state.IDLE)

        # After startup, transition to homing
        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

        # If homing is complete, transition to teleop
        elif self.state is self.state.HOMING:
            if self.arm.at_goal:
                self.transition_to(t_now, self.state.IDLE)

        # Idle mode
        elif self.state is self.state.IDLE:
            if arm_input.home:
                self.transition_to(t_now, self.state.HOMING)
                return

            if arm_input.lock_toggle and not arm_input.locked:
                self.phone_xyz_home = arm_input.phone_pos
                self.phone_rot_home = arm_input.phone_rot
                self.set_locked_pose(last_pos)
                self.transition_to(t_now, self.state.TELEOP)
                return

        # Teleop mode
        elif self.state is self.state.TELEOP:
            if arm_input.lock_toggle and arm_input.locked:
                g = hebi.arm.Goal(self.arm.size)
                g.add_waypoint(t=0.3,
                               position=self.arm.last_feedback.position_command,
                               velocity=np.zeros(g.dof_count),
                               acceleration=np.zeros(g.dof_count))
                self.arm.set_goal(g)
                self.transition_to(t_now, self.state.IDLE)
                return

            if (arm_goal := self.compute_arm_goal(arm_input)) is not None:
                self.arm.set_goal(arm_goal)

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

        elif state is self.state.IDLE:
            print(self.namespace + "TRANSITIONING TO IDLE")

        elif state is self.state.TELEOP:
            print(self.namespace + "TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print(self.namespace + "mobileIO timeout, disabling motion")

        elif state is self.state.EXIT:
            print(self.namespace + "TRANSITIONING TO EXIT")

        for handler in self._transition_handlers:
            handler(self, state)
        self.state = state

    def set_locked_pose(self, position):
        xyz = np.empty(3)
        orientation = np.empty((3, 3))
        self.arm.FK(position,
                    xyz_out=xyz,
                    orientation_out=orientation)

        self.last_locked_seed = position.copy()
        self.last_locked_xyz = xyz
        self.last_locked_rot = orientation

    def compute_arm_goal(self, arm_input: ArmMobileIOInputs):
        if np.any(np.isnan(arm_input.phone_pos)) or np.any(np.isnan(arm_input.phone_rot)):
            print(f'NaN detected in pose from AR device:\n{arm_input.phone_pos}\n{arm_input.phone_rot}')
            return None

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

        try:
            joint_target = self.arm.ik_target_xyz_so3(
                self.last_locked_seed,
                arm_xyz_target,
                arm_rot_target)
        except Exception as e:
            print('-----------------')
            print(arm_xyz_target)
            print('-----------------')
            print(self.last_locked_seed)
            print('-----------------')
            print(arm_rot_target)
            print('-----------------')
            raise e

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

    user_home = m.get_button_state(1)

    try:
        wxyz = m.orientation
        xyzw = [*wxyz[1:], wxyz[0]]
        rotation = R.from_quat(xyzw).as_matrix()
    except ValueError as e:
        print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
        rotation = np.eye(3)

    arm_input = ArmMobileIOInputs(
        home=user_home,
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
