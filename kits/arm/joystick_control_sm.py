import os
from enum import Enum, auto
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import time, sleep

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Sequence, Optional
    import numpy.typing as npt
    from hebi._internal.mobile_io import MobileIO


class ArmControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class ArmJoystickInputs:
    def __init__(self, home: bool, delta_xyz, delta_rot_xyz, gripper_closed: bool):
        self.home = home
        self.delta_xyz: 'npt.NDArray[np.float64]' = np.array(delta_xyz, dtype=np.float64)
        self.delta_rot_xyz: 'npt.NDArray[np.float64]' = np.array(delta_rot_xyz, dtype=np.float64)
        self.gripper_closed = gripper_closed


class ArmJoystickControl:
    def __init__(self, arm: hebi.arm.Arm, home_pose: 'Sequence[float] | npt.NDArray[np.float64]', homing_time: float = 5.0, shoulder_flip_angle=-np.pi / 2, joint_limits=None):
        self.state = ArmControlState.STARTUP
        self.arm = arm

        self.arm_home = home_pose
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

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.arm.send()

    def update(self, t_now: float, demo_input: 'Optional[ArmJoystickInputs]' = None):
        self.arm.update()

        if self.state is self.state.EXIT:
            return

        if demo_input is None:
            if t_now - self.mobile_last_fbk_t > 1.0 and self.state is not self.state.DISCONNECTED:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return

        self.mobile_last_fbk_t = t_now

        if self.state is self.state.DISCONNECTED:
            self.mobile_last_fbk_t = t_now
            self.transition_to(t_now, self.state.TELEOP)

        elif self.state is self.state.HOMING:
            if self.arm.at_goal:
                self.joint_target = self.arm.last_feedback.position_command
                self.transition_to(t_now, self.state.TELEOP)

        elif self.state is self.state.TELEOP:
            if demo_input.home:
                self.transition_to(t_now, self.state.HOMING)
                return

            arm_goal = self.compute_arm_goal(demo_input)
            self.arm.set_goal(arm_goal)

            gripper_closed = self.arm.end_effector.state == 1.0
            if demo_input.gripper_closed and not gripper_closed:
                self.arm.end_effector.close()
            elif not demo_input.gripper_closed and gripper_closed:
                self.arm.end_effector.open()

        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

    def transition_to(self, t_now: float, state: ArmControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print("TRANSITIONING TO HOMING")
            g = hebi.arm.Goal(self.arm.size)
            g.add_waypoint(t=self.homing_time, position=self.arm_home)
            self.arm.set_goal(g)

        elif state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO DISCONNECTED")

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        self.state = state

    def compute_arm_goal(self, arm_inputs: ArmJoystickInputs):
        arm_goal = hebi.arm.Goal(self.arm.size)
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

        r_x = R.from_euler('x', arm_inputs.delta_rot_xyz[0])
        r_y = R.from_euler('y', arm_inputs.delta_rot_xyz[1])
        r_z = R.from_euler('z', arm_inputs.delta_rot_xyz[2])
        wrist_rot = R.from_euler('z', -pos_curr[5])
        arm_rot_target = R.from_matrix(self.rot_curr) * wrist_rot * r_x * r_y * r_z * wrist_rot.inv()

        #curr_seed_ik = pos_curr
        #curr_seed_ik[2] = abs(curr_seed_ik[2])

        if self.joint_target[2] < self.shoulder_flip_angle:
            diff = abs(self.joint_target[2] - self.shoulder_flip_angle)
            self.joint_target[2] = self.shoulder_flip_angle + diff

        joint_target = self.arm.ik_target_xyz_so3(
            self.joint_target,
            arm_xyz_target,
            arm_rot_target.as_matrix())

        out_of_bounds = False
        for idx, joint in enumerate(joint_target):
            if joint < self.joint_limits[idx, 0] or joint > self.joint_limits[idx, 1]:
                out_of_bounds = True

        if not out_of_bounds:
            self.joint_target = joint_target

        arm_goal.add_waypoint(position=self.joint_target)
        return arm_goal


def setup_mobile_io(m: 'MobileIO'):
    m.resetUI()
    m.set_button_label(1, '⟲', blocking=False)
    m.set_button_label(2, '', blocking=False)
    m.set_button_label(3, '', blocking=False)
    m.set_button_label(4, 'Quit', blocking=False)
    m.set_button_label(6, '\u21E7', blocking=False)
    m.set_button_label(7, 'grip', blocking=False)
    m.set_button_mode(7, 1)
    m.set_button_label(8, '\u21E9', blocking=False)

    m.set_axis_label(3, '', blocking=False)
    m.set_axis_label(4, '', blocking=False)
    m.set_axis_label(5, '', blocking=False)
    m.set_axis_label(6, '', blocking=False)

    m.set_axis_label(1, '')
    m.set_axis_label(7, '')
    m.set_axis_label(2, 'rotate')
    m.set_axis_label(8, 'translate')
    m.set_axis_label(3, 'wrist', blocking=False)
    m.set_snap(3, 0)


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None

    home = m.get_button_state(1)

    arm_dx = 0.25 * m.get_axis_state(8)
    arm_dy = -0.25 * m.get_axis_state(7)

    arm_dz = 0.0
    if m.get_button_state(6):
        arm_dz = 0.1
    elif m.get_button_state(8):
        arm_dz = -0.1

    arm_drx = 0.5 * m.get_axis_state(1)
    arm_dry = -0.5 * m.get_axis_state(2)
    arm_drz = 0.75 * m.get_axis_state(3)

    gripper_closed = m.get_button_state(7)

    arm_inputs = ArmJoystickInputs(
        home,
        [arm_dx, arm_dy, arm_dz],
        [arm_drx, arm_dry, arm_drz],
        gripper_closed=gripper_closed)

    return arm_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Arm setup
    arm_family = "Arm"
    module_names = ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2']
    hrdf_file = "hrdf/A-2303-01G.hrdf"
    gains_file = "gains/A-2303-01.xml"

    root_dir = os.path.abspath(os.path.dirname(__file__))
    hrdf_file = os.path.join(root_dir, hrdf_file)
    gains_file = os.path.join(root_dir, gains_file)

    # Create Arm object
    arm = hebi.arm.create([arm_family],
                          names=module_names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)

    alt_shoulder_group = lookup.get_group_from_names(arm_family, ['J2B_shoulder1'])
    while not alt_shoulder_group:
        print(f"Looking for shoulder module {arm_family} / J2B_shoulder1 ...")
        sleep(1)
        alt_shoulder_group = lookup.get_group_from_names(arm_family, ['J2B_shoulder1'])

    double_shoulder = hebi.arm.DoubledJointMirror(1, alt_shoulder_group)
    arm.add_plugin(double_shoulder)

    arm.load_gains(gains_file)

    # Add the gripper
    gripper_family = arm_family
    gripper_name = 'gripperSpool'

    gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])
    while gripper_group is None:
        print(f"Looking for gripper module {gripper_family} / {gripper_name} ...")
        sleep(1)
        gripper_group = lookup.get_group_from_names([gripper_family], [gripper_name])

    gripper = hebi.arm.Gripper(gripper_group, -5, 1)
    gripper_gains = os.path.join(root_dir, "gains/gripper_spool_gains.xml")
    gripper.load_gains(gripper_gains)
    arm.set_end_effector(gripper)

    joint_limits = np.empty((7, 2))
    joint_limits[:, 0] = -np.inf
    joint_limits[:, 1] = np.inf

    # base limits [-2, 2] (radians)
    joint_limits[0, :] = [-2.0, 2.0]
    # shoulder limits [-2, inf]
    joint_limits[1, 0] = -2.0

    arm_control = ArmJoystickControl(arm,
                                     [0.0, -2.0, 0.0, -0.5, -1.5, 0.2, 0.0],
                                     homing_time=7.0,
                                     joint_limits=joint_limits)

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, arm_family)
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, arm_family)

    m.update()
    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    while arm_control.running:
        t = time()
        try:
            arm_inputs = parse_mobile_feedback(m)
            arm_control.update(t, arm_inputs)

            arm_control.send()
        except KeyboardInterrupt:
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')

        if m.get_button_state(4):
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')
