import hebi
from hebi.util import create_mobile_io
from time import time, sleep
import os
from .tready_utils import load_gains, set_mobile_io_instructions
from .tready import TreadedBase, TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity
import numpy as np

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def setup_mobile_io(m: 'MobileIO'):
    reset_pose_btn = 1
    torque_btn = 2
    rear_up_btn = 3
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
    roll_joy = 7 # Right Pad Left/Right
    pitch_joy = 8 # Right Pad Up/Down

    m.set_button_label(reset_pose_btn, '⟲', blocking=False)
    m.set_button_label(torque_btn, 'Torque', blocking=False)
    m.set_button_label(rear_up_btn, 'Rear Up', blocking=False)
    m.set_button_label(4, ' ', blocking=False)
    m.set_button_label(height_up_btn, '⤾◼⤿', blocking=False)
    m.set_button_label(recenter_btn, 'Center', blocking=False)
    m.set_button_label(height_down_btn, '⤿◼⤾', blocking=False)
    m.set_button_label(quit_demo_btn, '❌', blocking=False)

    m.set_button_mode(torque_btn, 1)
    # m.set_button_mode(rear_up_btn, 1)
    # m.set_button_output(torque_btn, 1)
    # m.set_button_output(rear_up_btn, 1)

    m.set_axis_label(turn_joy, 'Turn', blocking=False)
    m.set_axis_label(forward_joy, 'Drive', blocking=False)
    m.set_axis_label(front_left_slider, 'FL', blocking=False)
    m.set_axis_label(front_right_slider, 'FR', blocking=False)
    m.set_axis_label(back_left_slider, 'BL', blocking=False)
    m.set_axis_label(back_right_slider, 'BR', blocking=False)
    # m.set_axis_label(roll_joy, 'Roll', blocking=False)
    # m.set_axis_label(pitch_joy, 'Pitch', blocking=False)

    for i in range(1, 9):
        m.set_snap(i, 0)
    
    m.set_led_color('yellow')

    def parse_mobile_io_feedback(m: 'MobileIO') -> TreadyInputs:
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
                return True, None
            if m.get_button_state(reset_pose_btn):
                return False, TreadyInputs(home=True)
            if m.get_button_diff(torque_btn) == 1:
                change_to_torque_mode(m)
            elif m.get_button_diff(torque_btn) == -1:
                change_to_velocity_mode(m)
            if m.get_button_state(recenter_btn):
                return False, TreadyInputs(align_flippers=True)
            if m.get_button_state(rear_up_btn):
                pass
            
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

            return False, TreadyInputs(
                base_motion=chassis_velocity,
                flippers=flippers,
                torque_mode=m.get_button_state(torque_btn),
                torque_toggle=abs(m.get_button_diff(torque_btn))
            )

        else:
            return False, None
    
    def update_mobile_io(controller: TreadyControl, new_state: TreadyControlState):
        if controller.state == new_state:
            return

        if new_state is controller.state.HOMING:
            controller.base.set_color('magenta')
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(m, msg, color="blue")

        elif new_state is controller.state.ALIGNING:
            controller.base.set_color('magenta')
            msg = ('Robot Flippers Centering\n'
                   'Please wait...')
            set_mobile_io_instructions(m, msg, color="blue")

        elif new_state is controller.state.TELEOP:
            controller.base.clear_color()
            msg = ('Robot Ready to Control')
            set_mobile_io_instructions(m, msg, color="green")

        elif new_state is controller.state.DISCONNECTED:
            print('Lost connection to Controller. Please reconnect.')
            controller.base.set_color('blue')
        
        elif new_state is controller.state.EMERGENCY_STOP:
            controller.base.set_color('yellow')
            set_mobile_io_instructions(m, 'Emergency Stop Activated', color="red")

        elif new_state is controller.state.EXIT:
            controller.base.set_color("red")
            set_mobile_io_instructions(m, 'Demo Stopped', color="red")
    
    def update_torque_mode(controller: TreadyControl):
        if controller.state is controller.state.TELEOP:
            if controller.torque_labels is not None:
                for i, label in enumerate(controller.torque_labels):
                    m.set_axis_label(i+3, label, blocking=False)
            else:
                m.set_axis_label(front_left_slider, 'FL', blocking=False)
                m.set_axis_label(front_right_slider, 'FR', blocking=False)
                m.set_axis_label(back_left_slider, 'BL', blocking=False)
                m.set_axis_label(back_right_slider, 'BR', blocking=False)

    return parse_mobile_io_feedback, update_mobile_io, update_torque_mode


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    family = "Tready"
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # mobileIO setup
    print('Looking for mobileIO device...')
    m = create_mobile_io(lookup, family)
    while m is None:
        print('Waiting for mobileIO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, family)
    
    print("mobileIO device found.")
    m.update()
    parse_mobile_io_feedback, update_mobile_io, update_torque_mode = setup_mobile_io(m)

    # Create base group
    base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)
    while base_group is None:
        print('Looking for Tready modules...')
        sleep(1)
        base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)
    
    root_dir, _ = os.path.split(os.path.abspath(__file__))
    load_gains(base_group, os.path.join(root_dir, 'gains', 'smart-tready-gains.xml'))

    base = TreadedBase(base_group, chassis_ramp_time=0.33, flipper_ramp_time=0.1)
    base.set_robot_model(os.path.join(root_dir, 'hrdf', 'Tready.hrdf'))
    base_control = TreadyControl(base)

    base_control._transition_handlers.append(update_mobile_io)
    base_control._update_handlers.append(update_torque_mode)

    # can enable start logging here
    while base_control.running:
        t = time()
        try:
            quit, base_inputs = parse_mobile_io_feedback(m)
            if quit:
                break
            base_control.update(t, base_inputs)
            base_control.send()
        except KeyboardInterrupt:
            break
    
    base_control.stop()
    # stop logging here