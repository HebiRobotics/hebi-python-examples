import sys
from time import time, sleep

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

from kits.arm.ar_control_sm import ArmMobileIOControl, ArmControlState, ArmMobileIOInputs
from .tready import TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity
from .tready_utils import set_mobile_io_instructions, setup_base, setup_arm_7dof

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def setup_mobile_io(m: 'MobileIO'):
    """Sets up mobileIO interface.

    Return a function that parses mobileIO feedback into the format
    expected by the Demo
    """

    # MobileIO Button Config
    reset_pose_btn = 1
    joined_flipper_btn = 6
    quit_btn = 8

    slider_flip1 = 3
    slider_flip2 = 4
    slider_flip3 = 5
    slider_flip4 = 6

    joy_fwd = 2
    joy_rot = 1

    arm_enable = 2
    arm_lock = 4
    gripper_close = 5

    # set mobileIO control config
    m.set_led_color("blue")
    m.set_snap(slider_flip1, 0)
    m.set_snap(slider_flip2, 0)
    m.set_snap(slider_flip3, 0)
    m.set_snap(slider_flip4, 0)

    m.set_button_mode(joined_flipper_btn, 1)
    m.set_button_mode(arm_enable, 1)
    m.set_button_mode(arm_lock, 1)
    m.set_button_mode(gripper_close, 1)

    m.set_button_output(reset_pose_btn, 1)
    m.set_button_output(quit_btn, 1)

    m.set_button_output(arm_enable, 1)
    m.set_button_output(arm_lock, 1)

    def parse_mobile_io_feedback(m: 'MobileIO'):
        should_exit = m.get_button_state(quit_btn)
        should_reset = m.get_button_state(reset_pose_btn)
        # Chassis Control
        aligned_flipper_mode = m.get_button_state(joined_flipper_btn)
        joy_vel_fwd = m.get_axis_state(joy_fwd)
        joy_vel_rot = m.get_axis_state(joy_rot)

        # Flipper Control
        flip1 = m.get_axis_state(slider_flip1)
        flip2 = m.get_axis_state(slider_flip2)
        flip3 = m.get_axis_state(slider_flip3)
        flip4 = m.get_axis_state(slider_flip4)

        tready_inputs = TreadyInputs(
            should_reset,
            ChassisVelocity(joy_vel_fwd, joy_vel_rot),
            [flip1, flip2, flip3, flip4],
            aligned_flipper_mode)

        try:
            # reorder quaternion components
            wxyz = m.orientation
            xyzw = [*wxyz[1:4], wxyz[0]]
            rotation = R.from_quat(xyzw).as_matrix()
        except ValueError as e:
            print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
            rotation = np.eye(3)

        arm_inputs = ArmMobileIOInputs(
            np.copy(m.position),
            rotation,
            m.get_button_state(arm_lock),
            m.get_button_state(arm_enable),
            m.get_button_state(gripper_close))

        #return DemoInputs(should_exit, should_reset, tready_inputs, arm_inputs)
        return tready_inputs, arm_inputs

    return parse_mobile_io_feedback


if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    family = "Arm"

    arm = setup_arm_7dof(lookup, family)
    arm_control = ArmMobileIOControl(arm)

    # Base setup
    base = setup_base(lookup, family)
    base_control = TreadyControl(base)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for mobileIO device to come online...')
    m = create_mobile_io(lookup, family, phone_name)
    while m is None:
        m = create_mobile_io(lookup, family, phone_name)
        print("Could not find mobileIO device, waiting...")
        sleep(0.5)

    print("mobileIO device found.")
    parse_mobile_feedback = setup_mobile_io(m)

    def update_mobile_io(controller: TreadyControl, new_state: TreadyControlState):
        if controller.state == new_state:
            return

        if new_state is TreadyControlState.HOMING:
            controller.base.set_color('magenta')
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(m, msg)

        elif new_state is TreadyControlState.TELEOP:
            controller.base.clear_color()
            # Print Instructions
            instructions = ('Robot Ready to Control\n'
                            'B1: Reset\n'
                            'B2: Arm Motion Enable\n'
                            'B4: Arm Lock\n'
                            'B5: Close Gripper\n'
                            'B6: Joined Flipper\n'
                            'B8 - Quit')
            set_mobile_io_instructions(m, instructions, color='green')

        elif new_state is TreadyControlState.DISCONNECTED:
            controller.base.set_color('blue')

        elif new_state is TreadyControlState.EXIT:
            print("TRANSITIONING TO EXIT")
            controller.base.set_color("red")

            # unset mobileIO control config
            m.set_button_mode(6, 0)
            m.set_button_output(1, 0)
            m.set_button_output(8, 0)
            set_mobile_io_instructions(m, 'Demo Stopped.', color='red')

    base_control._transition_handlers.append(update_mobile_io)

    #######################
    ## Main Control Loop ##
    #######################

    m.set_led_color('blue')
    while base_control.running and arm_control.running:
        t = time()
        try:
            base_inputs, arm_inputs = parse_mobile_feedback(m)
            base_control.update(t, base_inputs)
            arm_control.update(t, arm_inputs)
        except KeyboardInterrupt:
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')

        if m.get_button_state(4):
            base_control.transition_to(t, TreadyControlState.EXIT)
            arm_control.transition_to(t, ArmControlState.EXIT)
            m.set_led_color('red')

        base_control.send()
        arm_control.send()

    sys.exit(0)
