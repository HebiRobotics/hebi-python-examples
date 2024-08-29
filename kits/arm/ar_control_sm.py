from enum import Enum, auto
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import time, sleep

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Optional
    import numpy.typing as npt
    from hebi._internal.mobile_io import MobileIO
    from hebi.arm import Arm


class ArmControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class ArmMobileIOInputs:
    def __init__(self, phone_pos: 'npt.NDArray[np.float32]' = np.array([0., 0., 0.]), phone_rot: 'npt.NDArray[np.float64]' = np.eye(3),
                 lock_toggle: bool = False, locked: bool = True, gripper_closed: bool = False, home: bool = False):

        self.phone_pos = phone_pos
        self.phone_rot = phone_rot
        self.lock_toggle = lock_toggle
        self.locked = locked
        self.gripper_closed = gripper_closed
        self.home = home


class ArmMobileIOControl:
    def __init__(self, arm: 'Arm'):
        self.namespace = ''

        self.state = ArmControlState.STARTUP
        self.arm = arm

        # 5 DoF home
        #self.arm_xyz_home = [0.34, 0.0, 0.23]
        # 6 DoF home
        self.arm_xyz_home = [0.5, 0.0, 0.0]
        self.arm_rot_home: 'npt.NDArray[np.float64]' = (R.from_euler('z', np.pi / 2) * R.from_euler('x', np.pi)).as_matrix()

        # 5 DoF seed
        #self.arm_seed_ik = np.array([0.25, -1.0, 0, -0.75, 0])
        # 6 DoF seed
        self.arm_seed_ik = np.array([0.3, 1.2, 2.2, 2.9, -1.57, 0])
        self.arm_home = self.arm.ik_target_xyz_so3(
            self.arm_seed_ik,
            self.arm_xyz_home,
            self.arm_rot_home)
        
        self.last_locked_xyz = self.arm_xyz_home.copy()
        self.last_locked_rot = self.arm_rot_home.copy()
        self.last_locked_seed = self.arm_home.copy()

        self.locked = True

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.arm.send()

    def update(self, t_now: float, arm_input: 'Optional[ArmMobileIOInputs]'):
        self.arm.update()
        self.arm.send()

        if self.state is self.state.EXIT:
            return
        
        if arm_input is None and self.state is not self.state.DISCONNECTED:
            if t_now - self.mobile_last_fbk_t > 1.0:
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
                self.phone_xyz_home = arm_input.phone_pos
                self.phone_rot_home = arm_input.phone_rot
                
                self.last_locked_seed = np.array(self.arm.last_feedback.position)
                xyz = np.zeros(3)
                orientation = np.zeros((3,3))
                self.arm.FK(self.arm.last_feedback.position, xyz_out=xyz, orientation_out=orientation)

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

                self.last_locked_seed = np.array(self.arm.last_feedback.position)
                xyz = np.zeros(3)
                orientation = np.zeros((3,3))
                self.arm.FK(self.arm.last_feedback.position, xyz_out=xyz, orientation_out=orientation)

                self.last_locked_xyz = xyz
                self.last_locked_rot = orientation

            gripper = self.arm.end_effector
            if gripper is not None:
                gripper_closed = gripper.state == 1.0
                if arm_input.gripper_closed and not gripper_closed:
                    gripper.close()
                elif not arm_input.gripper_closed and gripper_closed:
                    gripper.open()

    def transition_to(self, t_now: float, state: ArmControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print(self.namespace + "TRANSITIONING TO HOMING")
            self.home()

        elif state is self.state.TELEOP:
            print(self.namespace + "TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print(self.namespace + "mobileIO timeout, disabling motion")

        elif state is self.state.EXIT:
            print(self.namespace + "TRANSITIONING TO EXIT")

        self.state = state

    def compute_arm_goal(self, arm_input: ArmMobileIOInputs):
        xyz_scale = np.array([1.0, 1.0, 1.0])

        phone_offset = arm_input.phone_pos - self.phone_xyz_home
        rot_mat = self.phone_rot_home
        arm_xyz_target = self.last_locked_xyz + xyz_scale * (rot_mat.T @ phone_offset)
        arm_rot_target = rot_mat.T @ arm_input.phone_rot @ self.last_locked_rot
        
        joint_target = self.arm.ik_target_xyz_so3(
            self.last_locked_seed,
            arm_xyz_target,
            arm_rot_target)

        arm_goal = hebi.arm.Goal(self.arm.size)
        arm_goal.add_waypoint(position=joint_target)
        return arm_goal
    
    def home(self):
        g = hebi.arm.Goal(self.arm.size)
        g.add_waypoint(position=self.arm_home)
        self.arm.set_goal(g)
        
    def stop(self):
        self.transition_to(time(), self.state.EXIT)


def setup_mobile_io(m: 'MobileIO'):
    m.set_button_label(1, '⟲', blocking=False)
    m.set_button_label(5, 'lock', blocking=False)
    m.set_button_mode(5, 1, blocking=False)
    m.set_button_label(7, 'grip', blocking=False)
    m.set_button_mode(7, 1, blocking=False)
    m.set_button_label(8, '❌', blocking=False)


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None
    
    if m.get_button_state(8):
        return True, None

    if m.get_button_state(1):
        return False, ArmMobileIOInputs(home=True)
    
    try:
        rotation = R.from_quat(m.orientation, scalar_first=True).as_matrix()
    except ValueError as e:
        print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
        rotation = np.eye(3)

    arm_input = ArmMobileIOInputs(
        np.copy(m.position),
        rotation,
        m.get_button_state(5),
        m.get_button_state(7))

    return False, arm_input

if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Arm setup
    arm_family = "T-arm"
    module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
    hrdf_file = "hrdf/A-2085-06.hrdf"
    gains_file = "gains/A-2085-06.xml"

    # Create Arm object
    arm = hebi.arm.create([arm_family],
                          names=module_names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)
    arm.load_gains(gains_file)

    arm_control = ArmMobileIOControl(arm)

    # Setup MobileIO
    print('Looking for mobileIO device...')
    m = create_mobile_io(lookup, arm_family)
    while m is None:
        print('Waiting for mobileIO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, arm_family)
    
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