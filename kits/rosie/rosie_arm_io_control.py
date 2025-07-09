import sys
from time import time, sleep
import os
import datetime
import warnings

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

from kits.arms.ar_control_sm import ArmMobileIOControl, ArmControlState, ArmMobileIOInputs
from ..bases.omni_base import ChassisVelocity, OmniBase
from .rosie_utils import set_mobile_io_instructions, setup_base, setup_arm

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def mio_layout_updater(mio: 'MobileIO'):

    def update_mobile_io(controller: ArmMobileIOControl, new_state: ArmControlState):
        if controller.state == new_state:
            return

        if new_state is new_state.HOMING:
            controller.arm.pending_command.led.color = 'magenta'
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(mio, msg)

        elif new_state is new_state.TELEOP:
            controller.arm.pending_command.led.color = 'transparent'
            # Print Instructions
            set_mobile_io_instructions(
                mio, 'Robot Ready to Control', color='green')

        elif new_state is new_state.DISCONNECTED:
            controller.arm.pending_command.led.color = 'blue'

        elif new_state is new_state.EXIT:
            print("TRANSITIONING TO EXIT")
            controller.arm.pending_command.led.color = 'transparent'

            # unset mobileIO control config
            mio.resetUI()
            set_mobile_io_instructions(mio, 'Demo Stopped.', color='red')
    return update_mobile_io


class RosieControl:
    def __init__(self, base: 'OmniBase'):
        self.base = base
        self.running = True
        self.on_shutdown = lambda: None

    def update(self, t_now, base_inputs: 'ChassisVelocity | None'):
        self.base.update(t_now)
        if base_inputs is None:
            return
        self.base.build_smooth_velocity_trajectory(
            base_inputs.x, base_inputs.y, base_inputs.rz, t_now)

    def send(self):
        self.base.send()

    def stop(self):
        self.running = False
        self.base.base_command.velocity = np.zeros(3)
        self.on_shutdown()


def setup_mobile_io(m: 'MobileIO'):
    """Sets up mobileIO interface.

    Return a function that parses mobileIO feedback into the format
    expected by the Demo
    """


    # Base
    reset_pose_btn = 1
    quit_demo_btn = 8

    side_joy = 1  # Left Pad Left/Right
    forward_joy = 2  # Left Pad Up/Down
    ar_xyz_scale_slider = 4
    turn_joy = 7  # Right Pad Left/Right

    m.resetUI()

    m.set_button_label(reset_pose_btn, '⟲', blocking=False)
    m.set_button_label(quit_demo_btn, '❌', blocking=False)

    m.set_axis_label(side_joy, '', blocking=False)
    m.set_axis_label(forward_joy, 'Translate', blocking=False)
    m.set_axis_label(ar_xyz_scale_slider, 'XYZ\nScale', blocking=False)
    m.set_axis_label(turn_joy, '', blocking=False)
    m.set_axis_label(8, 'Rotate', blocking=False)

    for i, snap in enumerate([0, 0, 0, np.nan, 0, 0, 0, 0]):
        m.set_snap(i+1, snap)

    global ar_scaling
    ar_scaling = 1.0
    m.set_axis_value(ar_xyz_scale_slider, ar_scaling)

    m.set_led_color('yellow')

    # Arm
    arm_lock = 3
    gripper_close = 4

    m.set_button_label(arm_lock, 'Arm 🔒', blocking=False)
    m.set_button_label(gripper_close, 'Gripper', blocking=False)
    m.set_button_mode(arm_lock, 1)
    m.set_button_mode(gripper_close, 1)
    m.set_button_output(arm_lock, 1)
    m.set_button_output(gripper_close, 1)

    def parse_mobile_io_feedback(m: 'MobileIO'):
        if not m.update(0.0):
            return False, None, None
        else:
            if m.get_button_state(quit_demo_btn):
                return True, None, None
            if m.get_button_state(reset_pose_btn):
                return False, ChassisVelocity(), ArmMobileIOInputs(home=True)

            chassis_velocity = ChassisVelocity(
                pow(m.get_axis_state(forward_joy), 3),
                pow(-m.get_axis_state(side_joy), 3),
                pow(-m.get_axis_state(turn_joy), 3) * 2
            )

            try:
                rotation = R.from_quat(
                    m.orientation, scalar_first=True).as_matrix()
            except ValueError as e:
                print(
                    f'Error getting orientation as matrix: {e}\n{m.orientation}')
                rotation = np.eye(3)

            if m.get_button_diff(arm_lock) != 0:
                if not m.get_button_state(arm_lock):
                    m.set_button_label(arm_lock, 'Arm 🔒', blocking=False)
                elif m.get_button_state(arm_lock):
                    m.set_button_label(arm_lock, 'Arm 🔓', blocking=False)


            locked = m.get_button_state(arm_lock)
            global ar_scaling
            if locked:
                m.set_axis_value(ar_xyz_scale_slider, (2 * ar_scaling) - 1.0)
            if not locked:
                ar_scaling = (m.get_axis_state(ar_xyz_scale_slider) + 1.0) / 2.0
                if ar_scaling < 0.1:
                    ar_scaling = 0.0

            arm_inputs = ArmMobileIOInputs(
                phone_pos=np.copy(m.position),
                phone_rot=rotation,
                ar_scaling=ar_scaling,
                lock_toggle=(m.get_button_diff(arm_lock) != 0),
                locked=(not locked),
                gripper_closed=m.get_button_state(gripper_close)
            )

            return False, chassis_velocity, arm_inputs

    return parse_mobile_io_feedback


if __name__ == "__main__":

    lookup = hebi.Lookup()

    root_dir = os.path.dirname(__file__)
    cfg_file = os.path.join(root_dir, 'config', 'rosie-t.cfg.yaml')
    cfg = hebi.config.load_config(cfg_file)
    family = cfg.families[0]

    # mobileIO setup
    print('Waiting for mobileIO device to come online...')
    while (m := create_mobile_io(lookup, family)) is None:
        print("Could not find mobileIO device, waiting...")
        sleep(0.5)

    print("mobileIO device found.")
    parse_mobile_feedback = setup_mobile_io(m)

    # Base setup
    base = setup_base(lookup, family)
    base_control = RosieControl(base)

    tries = 3
    arm, gripper = setup_arm(cfg, lookup)
    while arm is None and tries > 0:
        sleep(1)
        tries -= 1
        arm, gripper = setup_arm(cfg, lookup)

    if arm is None:
        warnings.warn('Could not find arm! Continuing without...')
        arm_control = None
    else:
        while not arm.update():
            print('Waiting for feedback from arm...')
            sleep(1.0)

        args = []
        if user_data := cfg.user_data:
            args.append(user_data['homing_duration'])
            args.append(user_data['delay_time'])
            args.append(np.array(user_data['xyz_scale']))

        arm_control = ArmMobileIOControl(arm, gripper, *args)

    if arm_control:
        arm_control._transition_handlers.append(mio_layout_updater(m))
    else:
        set_mobile_io_instructions(m, 'Robot Ready to Control', color='green')

        def on_shutdown():
            if m:
                set_mobile_io_instructions(m, 'Demo Stopped.', color='red')
        base_control.on_shutdown = on_shutdown

    #######################
    ## Main Control Loop ##
    #######################

    logging = True

    if logging:
        rosie_dir = os.path.dirname(__file__)
        now = datetime.datetime.now()
        base.group.start_log(os.path.join(rosie_dir, 'logs'),
                             f'base_{now:%Y-%m-%d-%H:%M:%S}', mkdirs=True)
        if arm:
            arm.group.start_log(os.path.join(rosie_dir, 'logs'),
                                f'arm_{now:%Y-%m-%d-%H:%M:%S}', mkdirs=True)

    while base_control.running and (arm_control is None or arm_control.running):
        t = time()
        try:
            quit, base_inputs, arm_inputs = parse_mobile_feedback(m)
            base_control.update(t, base_inputs)
            if arm_control:
                arm_control.update(t, arm_inputs)
        except KeyboardInterrupt:
            quit = True

        if quit:
            base_control.stop()
            if arm_control:
                arm_control.transition_to(t, ArmControlState.EXIT)

        base_control.send()
        if arm_control:
            arm_control.send()

    if logging:
        base.group.stop_log()
        if arm:
            arm.group.stop_log()

    sys.exit(0)
