import hebi
from hebi.util import create_mobile_io
from time import time, sleep
import datetime
import os
import sys
from .tready_utils import load_gains, set_mobile_io_instructions, setup_arm_6dof
from .tready import TreadedBase, TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity
from kits.arms.ar_control_sm import ArmMobileIOControl, ArmMobileIOInputs
import numpy as np
from scipy.spatial.transform import Rotation as R


import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def setup_mobile_io(m: 'MobileIO'):
    """Sets up mobileIO interface.

    Return a function that parses mobileIO feedback into the format
    expected by the Demo
    """

    # Base
    reset_pose_btn = 1
    torque_btn = 2
    height_up_btn = 5
    recenter_btn = 6
    height_down_btn = 7
    quit_demo_btn = 8

    turn_joy = 1 # Left Pad Left/Right
    forward_joy = 2 # Left Pad Up/Down
    front_left_slider = 3
    front_right_slider = 4
    back_left_slider = 5
    back_right_slider = 6

    m.set_button_label(reset_pose_btn, '⟲', blocking=False)
    m.set_button_label(torque_btn, 'Torque', blocking=False)
    m.set_button_label(height_up_btn, '⤾◼⤿', blocking=False)
    m.set_button_label(recenter_btn, 'Center', blocking=False)
    m.set_button_label(height_down_btn, '⤿◼⤾', blocking=False)
    m.set_button_label(quit_demo_btn, '❌', blocking=False)

    m.set_button_mode(torque_btn, 1)

    m.set_axis_label(turn_joy, 'Turn', blocking=False)
    m.set_axis_label(forward_joy, 'Drive', blocking=False)
    m.set_axis_label(front_left_slider, 'FL', blocking=False)
    m.set_axis_label(front_right_slider, 'FR', blocking=False)
    m.set_axis_label(back_left_slider, 'BL', blocking=False)
    m.set_axis_label(back_right_slider, 'BR', blocking=False)

    for i in range(1, 9):
        m.set_snap(i, 0)
    
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
        def change_to_torque_mode(m: 'MobileIO'):
            axis_vals = [0, -0.5, 1, 1]
            for i in range(3, 7):
                if not m.set_snap(i, np.nan):
                    print(f'Failed to set snap for axis {i}')
                if not m.set_axis_value(i, axis_vals[i-3]):
                    print(f'Failed to set axis value for axis {i}')
        
        def change_to_velocity_mode(m: 'MobileIO'):
            for i in range(3, 7):
                if not m.set_snap(i, 0):
                    print(f'Failed to set snap for axis {i}')
            m.set_axis_label(front_left_slider, 'FL', blocking=False)
            m.set_axis_label(front_right_slider, 'FR', blocking=False)
            m.set_axis_label(back_left_slider, 'BL', blocking=False)
            m.set_axis_label(back_right_slider, 'BR', blocking=False)
        
        if m.update(0.0):
            if m.get_button_state(quit_demo_btn):
                return True, None, None
            if m.get_button_state(reset_pose_btn):
                return False, TreadyInputs(home=True, torque_mode=m.get_button_state(torque_btn), torque_toggle=abs(m.get_button_diff(torque_btn))), ArmMobileIOInputs(home=True)
            
            if m.get_button_diff(torque_btn) == 1:
                change_to_torque_mode(m)
            elif m.get_button_diff(torque_btn) == -1:
                change_to_velocity_mode(m)
            
            tready_inputs = None
            if m.get_button_state(recenter_btn):
                tready_inputs = TreadyInputs(align_flippers=True, torque_mode=m.get_button_state(torque_btn), torque_toggle=abs(m.get_button_diff(torque_btn)))
            else:
                chassis_velocity = ChassisVelocity(
                    m.get_axis_state(forward_joy),
                    m.get_axis_state(turn_joy)
                )
                height_up = m.get_button_state(height_up_btn)
                height_down = m.get_button_state(height_down_btn)
                height = height_up - height_down
                if height != 0:
                    flippers = [-height/2] * 4
                else:
                    flippers = [
                        m.get_axis_state(front_left_slider),
                        m.get_axis_state(front_right_slider),
                        m.get_axis_state(back_left_slider),
                        m.get_axis_state(back_right_slider)
                    ]

                tready_inputs = TreadyInputs(
                    base_motion=chassis_velocity,
                    flippers=flippers,
                    torque_mode=m.get_button_state(torque_btn),
                    torque_toggle=abs(m.get_button_diff(torque_btn))
                )

            try:
                rotation = R.from_quat(m.orientation, scalar_first=True).as_matrix()
            except ValueError as e:
                print(f'Error getting orientation as matrix: {e}\n{m.orientation}')
                rotation = np.eye(3)

            if m.get_button_diff(arm_lock) != 0:
                if not m.get_button_state(arm_lock):
                    m.set_button_label(arm_lock, 'Arm 🔒', blocking=False)
                elif m.get_button_state(arm_lock):
                    m.set_button_label(arm_lock, 'Arm 🔓', blocking=False)

            arm_inputs = ArmMobileIOInputs(
                np.copy(m.position),
                rotation,
                bool(m.get_button_diff(arm_lock) != 0),
                not m.get_button_state(arm_lock),
                m.get_button_state(gripper_close)
            )
            
            return False, tready_inputs, arm_inputs

        else:
            return False, None, None

    return parse_mobile_io_feedback


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    base_family = "Tready"
    arm_family = "Arm"
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # mobileIO setup
    print('Looking for mobileIO device...')
    m = create_mobile_io(lookup, base_family)
    search_start = time()
    while m is None:
        if time() - search_start > 30.0:
            print("Couldn't find tablet, restarting...")
            sys.exit()
        print('Waiting for mobileIO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, base_family)
    
    print("mobileIO device found.")
    m.update()
    parse_mobile_io_feedback = setup_mobile_io(m)

    # Create Arm group
    arm = setup_arm_6dof(lookup, arm_family, with_gripper=False)
    if arm is not None:
        arm_control = ArmMobileIOControl(arm)
        arm_control.namespace = "[Arm] "

    # Create base group
    base_group = lookup.get_group_from_names(base_family, wheel_names + flipper_names)
    while base_group is None:
        print('Looking for Tready modules...')
        sleep(1)
        base_group = lookup.get_group_from_names(base_family, wheel_names + flipper_names)
    
    root_dir, _ = os.path.split(os.path.abspath(__file__))
    load_gains(base_group, os.path.join(root_dir, 'gains', 'smart-tready-gains.xml'))

    base = TreadedBase(base_group, chassis_ramp_time=0.33, flipper_ramp_time=0.1)
    base.set_robot_model(os.path.join(root_dir, 'hrdf', 'Tready.hrdf'))
    base_control = TreadyControl(base)
    base_control.namespace = "[Base] "

    def update_mobile_io(controller: TreadyControl, new_state: TreadyControlState):
        if controller.state == new_state:
            return

        if new_state is TreadyControlState.HOMING:
            controller.base.set_color('magenta')
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(m, msg, color="blue")

        elif new_state is TreadyControlState.ALIGNING:
            controller.base.set_color('magenta')
            msg = ('Robot Flippers Centering\n'
                   'Please wait...')
            set_mobile_io_instructions(m, msg, color="blue")

        elif new_state is TreadyControlState.TELEOP:
            controller.base.set_color('transparent')
            msg = ('Robot Ready to Control')
            set_mobile_io_instructions(m, msg, color="green")

        elif new_state is TreadyControlState.DISCONNECTED:
            print('Lost connection to Controller. Please reconnect.')
            controller.base.set_color('blue')
        
        elif new_state is TreadyControlState.EMERGENCY_STOP:
            controller.base.set_color('yellow')
            set_mobile_io_instructions(m, 'Emergency Stop Activated', color="red")

        elif new_state is TreadyControlState.EXIT:
            controller.base.set_color("red")
            set_mobile_io_instructions(m, 'Demo Stopped', color="red")
    
    def update_torque_mode(controller: TreadyControl):
        if controller.state is TreadyControlState.TELEOP:
            if controller.torque_labels is not None:
                for i, label in enumerate(controller.torque_labels):
                    m.set_axis_label(i+3, label, blocking=False)
            else:
                m.set_axis_label(3, 'FL', blocking=False)
                m.set_axis_label(4, 'FR', blocking=False)
                m.set_axis_label(5, 'BL', blocking=False)
                m.set_axis_label(6, 'BR', blocking=False)
    
    base_control._transition_handlers.append(update_mobile_io)
    base_control._update_handlers.append(update_torque_mode)

    logging = True

    if logging:
        tready_dir = os.path.dirname(__file__)
        now = datetime.datetime.now()
        if arm:
            arm.group.start_log(os.path.join(tready_dir, 'logs'), f'arm_{now:%Y-%m-%d-%H:%M:%S}')
        base.group.start_log(os.path.join(tready_dir, 'logs'), f'base_{now:%Y-%m-%d-%H:%M:%S}')

    voltage = np.mean(base_control.base.fbk.voltage)
    while base_control.running and (arm is None or arm_control.running):
        t = time()
        try:
            quit, base_inputs, arm_inputs = parse_mobile_io_feedback(m)
            if quit:
                break
            if base_control.state is TreadyControlState.TELEOP:
                new_voltage = np.mean(base_control.base.fbk.voltage)
                if abs(voltage - new_voltage) > 0.1:
                    m.clear_text()
                    if new_voltage < 31:
                        msg = f'CHARGE NOW! {voltage:.2f}V'
                    elif new_voltage < 32:
                       msg = f'Low Battery: {voltage:.2f}V'
                    else:
                       msg = f'Battery: {voltage:.2f}V'
                    m.add_text(msg)
                voltage = new_voltage
            base_control.update(t, base_inputs)
            if arm is not None:
                arm_control.update(t, arm_inputs)
            base_control.send()
            if arm is not None:
                arm_control.send()
        except KeyboardInterrupt:
            break
    
    base_control.stop()
    if arm is not None:
        arm_control.stop()

    if logging:
        arm.group.stop_log()
        base.group.stop_log()