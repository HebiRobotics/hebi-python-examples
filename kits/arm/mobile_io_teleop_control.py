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
    def __init__(self,
                 m: 'MobileIO',
                 num_joints: 'int',
                 locked: bool,
                 active: bool,
                 gripper_closed: bool):

        self.mio = m
        
        self.num_joints = num_joints

        self.joint_dt = 0.2
        self.joint_velocity_max = 1.0
        self.joint_velocity = np.zeros(self.num_joints)
        self.prev_joint_velocity = np.zeros(self.num_joints)
        self.joint_displacement = np.zeros(self.num_joints)
        self.joint_direction = 1.0
        self.alpha_joint = 0.1

        self.cartesian_dt = 0.2
        self.cartesian_velocity_max = 0.25
        self.cartesian_velocity = np.zeros(3)
        self.prev_cartesian_velocity = np.zeros(3)
        self.cartesian_displacement = np.zeros(3)
        self.alpha_cartesian = 0.2

        self.locked = locked
        self.active = active
        self.gripper_closed = gripper_closed
    
    def clear_input(self):
        self.joint_velocity = np.zeros(self.num_joints)
        self.cartesian_velocity = np.zeros(3)
    
    def parse_mobile_feedback(self):
        if not self.mio.update(0.0):
            self.clear_input()
        
        self.joint_direction = -(int(self.mio.get_button_state(7)) * 2 - 1)

        for i in range(self.num_joints):
            self.joint_velocity[i] = int(self.mio.get_button_state(i+1)) * self.joint_velocity_max * self.joint_direction

        self.cartesian_velocity[0] = self.mio.get_axis_state(8) * self.cartesian_velocity_max
        self.cartesian_velocity[1] = -self.mio.get_axis_state(7) * self.cartesian_velocity_max
        self.cartesian_velocity[2] = self.mio.get_axis_state(5) * self.cartesian_velocity_max

        self.prev_cartesian_velocity = self.cartesian_velocity * self.alpha_cartesian + self.prev_cartesian_velocity * (1.0 - self.alpha_cartesian)
        self.prev_joint_velocity = self.joint_velocity * self.alpha_joint + self.prev_joint_velocity * (1.0 - self.alpha_joint)
    
    def get_arm_inputs(self):
        if np.linalg.norm(self.cartesian_displacement) > 1e-3:
            return self.prev_cartesian_velocity * self.cartesian_dt
        elif np.linalg.norm(self.prev_joint_velocity) > 1e-2:
            return self.prev_joint_velocity * self.joint_dt
        else:
            return np.zeros(self.num_joints)


class ArmControl:
    def __init__(self, arm: 'Arm'):
        self.state = ArmControlState.STARTUP
        self.arm = arm

        # 5 DoF home
        #self.arm_xyz_home = [0.34, 0.0, 0.23]
        # 6 DoF home
        self.arm_xyz_home = [0.4, 0.0, 0.0]
        self.arm_rot_home: 'npt.NDArray[np.float64]' = (R.from_euler('z', np.pi / 2) * R.from_euler('x', np.pi)).as_matrix()

        # 5 DoF seed
        #self.arm_seed_ik = np.array([0.25, -1.0, 0, -0.75, 0])
        # 6 DoF seed
        self.arm_seed_ik = np.array([0, 0.5, 2, 3, -1.5, 0])
        self.arm_home = self.arm.ik_target_xyz_so3(
            self.arm_seed_ik,
            self.arm_xyz_home,
            self.arm_rot_home)

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.arm.send()

    def update(self, t_now: float, arm_input: 'Optional[ArmMobileIOInputs]'):
        self.arm.update()
        self.arm.send()

        if self.state is self.state.EXIT:
            return False

        if not arm_input:
            if t_now - self.last_input_time > 1.0 and self.state is not self.state.DISCONNECTED:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return True
        else:
            self.last_input_time = t_now

            if self.state is self.state.DISCONNECTED:
                self.last_input_time = t_now
                self.transition_to(t_now, self.state.TELEOP)
                return True

            elif self.state is self.state.HOMING:
                if self.arm.at_goal:
                    # self.phone_xyz_home = arm_input.phone_pos
                    # self.phone_rot_home = arm_input.phone_rot
                    self.transition_to(t_now, self.state.TELEOP)
                return True

            elif self.state is self.state.TELEOP:
                if arm_input.locked:
                    self.transition_to(t_now, self.state.HOMING)
                else:
                    arm_goal = self.compute_arm_goal(arm_input)
                    if arm_goal is not None:
                        self.arm.set_goal(arm_goal)

                gripper = self.arm.end_effector
                if gripper is not None:
                    gripper_closed = gripper.state == 1.0
                    if arm_input.gripper_closed and not gripper_closed:
                        gripper.close()
                    elif not arm_input.gripper_closed and gripper_closed:
                        gripper.open()

                return True

            elif self.state is self.state.STARTUP:
                self.transition_to(t_now, self.state.HOMING)
                return True

    def transition_to(self, t_now: float, state: ArmControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print("TRANSITIONING TO HOMING")
            g = hebi.arm.Goal(self.arm.size)
            g.add_waypoint(position=self.arm_home)
            self.arm.set_goal(g)

        elif state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        self.state = state

    def compute_arm_goal(self, arm_inputs: ArmMobileIOInputs):
        xyz_scale = np.array([1.0, 1.0, 1.0])

        phone_xyz = arm_inputs.phone_pos
        phone_rot = arm_inputs.phone_rot

        if not arm_inputs.active:
            # update phone "zero"
            self.phone_xyz_home = phone_xyz
            self.phone_rot_home = phone_rot
            return None
        else:
            phone_offset = phone_xyz - self.phone_xyz_home
            rot_mat = self.phone_rot_home
            arm_xyz_target = self.arm_xyz_home + xyz_scale * (rot_mat.T @ phone_offset)
            arm_rot_target = rot_mat.T @ phone_rot @ self.arm_rot_home

            joint_target = self.arm.ik_target_xyz_so3(
                self.arm_seed_ik,
                arm_xyz_target,
                arm_rot_target)

            arm_goal = hebi.arm.Goal(self.arm.size)
            arm_goal.add_waypoint(position=joint_target)
            return arm_goal


def setup_mobile_io(m: 'MobileIO'):
    m.set_button_label(1, '⟲')
    m.set_button_label(5, 'arm')
    m.set_button_mode(5, 1)
    m.set_button_label(7, 'grip')
    m.set_button_mode(7, 1)





if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Arm setup
    arm_family = "Arm"
    module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
    hrdf_file = "hrdf/A-2085-06.hrdf"
    gains_file = "gains/A-2085-06.xml"

    # Create Arm object
    arm = hebi.arm.create([arm_family],
                          names=module_names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)
    arm.load_gains(gains_file)

    arm_control = ArmControl(arm)

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
        arm_inputs = parse_mobile_feedback(m)
        arm_control.update(t, arm_inputs)

        arm_control.send()
