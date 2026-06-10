import hebi
from hebi.util import create_mobile_io
from time import time, sleep
import datetime
import os
from os.path import join
from .treadward_utils import load_gains, set_mobile_io_instructions
from .treadward import TreadedBase, TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity
from .treadward_sampler import CoreSampler, CoreSamplerControl, CoreSamplerControlState, CoreSamplerInputs
import numpy as np
from enum import Enum, auto

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO
    from typing import Optional, Callable

layout_dir = join(os.path.dirname(__file__), 'config', 'layouts')
drive_layout = 'TreadwardDriveController.json'
deployed_layout = 'TreadwardDeployedController.json'
deploying_layout = "TreadwardDeployingController.json"

class MobileIOModes(Enum):
    DRIVE = auto()
    DEPLOYING = auto()
    DEPLOYED = auto()

mobileIO_mode = MobileIOModes.DRIVE


def parse_mobile_io_feedback(m: 'MobileIO', io_mode: 'MobileIOModes'):

    def update_drive_mode(m: 'MobileIO'):
        # Drive buttons, joysticks, and sliders
        mapping = {
            'reset_pose_btn': 1,
            'torque_btn': 2,
            'deploy_btn': 3,
            'height_up_btn': 5,
            'recenter_btn': 6,
            'height_down_btn': 7,
            'quit_demo_btn': 8,
            'turn_joy': 1,
            'forward_joy': 2,
            'front_left_slider': 3,
            'front_right_slider': 4,
            'back_left_slider': 5,
            'back_right_slider': 6
        }   

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
            m.set_axis_label(mapping['front_left_slider'], 'FL', blocking=False)
            m.set_axis_label(mapping['front_right_slider'], 'FR', blocking=False)
            m.set_axis_label(mapping['back_left_slider'], 'BL', blocking=False)
            m.set_axis_label(mapping['back_right_slider'], 'BR', blocking=False)
        
        if m.get_button_state(mapping['quit_demo_btn']):
            print("QUIT PLZZZZ")
            return True, None, None
        if m.get_button_state(mapping['reset_pose_btn']):
            return False, TreadyInputs(home=True, torque_mode=m.get_button_state(mapping['torque_btn']), torque_toggle=(m.get_button_diff(mapping['torque_btn']) != 0.0)), None
        if m.get_button_diff(mapping['torque_btn']) == 1:
            change_to_torque_mode(m)
        elif m.get_button_diff(mapping['torque_btn']) == -1:
            change_to_velocity_mode(m)
        if m.get_button_state(mapping['recenter_btn']):
            return False, TreadyInputs(align_flippers=True, torque_mode=m.get_button_state(mapping['torque_btn']), torque_toggle=(m.get_button_diff(mapping['torque_btn']) != 0.0)), None
        if m.get_button_state(mapping['deploy_btn']):
            b_inputs = TreadyInputs(deploy=True, torque_mode=m.get_button_state(mapping['torque_btn']), torque_toggle=(m.get_button_diff(mapping['torque_btn']) != 0.0))
            p_inputs = CoreSamplerInputs(deploy=True)
            return False, b_inputs, p_inputs

        chassis_velocity = ChassisVelocity(
            m.get_axis_state(mapping['forward_joy']),
            m.get_axis_state(mapping['turn_joy'])
        )
        height_up = m.get_button_state(mapping['height_up_btn'])
        height_down = m.get_button_state(mapping['height_down_btn'])
        height = height_up - height_down
        if height != 0:
            flippers = [-height/2] * 4
        else:
            flippers = [
                m.get_axis_state(mapping['front_left_slider']),
                m.get_axis_state(mapping['front_right_slider']),
                m.get_axis_state(mapping['back_left_slider']),
                m.get_axis_state(mapping['back_right_slider'])
            ]

        return False, TreadyInputs(
            base_motion=chassis_velocity,
            flippers=flippers,
            torque_mode=m.get_button_state(mapping['torque_btn']),
            torque_toggle=(m.get_button_diff(mapping['torque_btn']) != 0.0),
            ), None

    def update_deployed_mode(m: 'MobileIO'):
        # Deployed buttons and sliders
        mapping = {
            'stow_btn': 3,
            'quit_demo_btn': 8,
            'chain_slider': 6,
            'wiggle_btn': 2
        }

        if m.get_button_state(mapping['quit_demo_btn']):
            return True, None, None
        
        b_inputs = TreadyInputs(stow=m.get_button_state(mapping['stow_btn']))
        p_inputs = CoreSamplerInputs(stow=m.get_button_state(mapping['stow_btn']), 
                                        chain=m.get_axis_state(mapping["chain_slider"]), 
                                        wiggle_mode=m.get_button_state(mapping["wiggle_btn"]),
                                        wiggle_toggle=(m.get_button_diff(mapping["wiggle_btn"]) != 0))
        return False, b_inputs, p_inputs

    def update_deploying_mode(m: 'MobileIO'): 
        # Deploying buttons and sliders
        mapping = {
            'quit_demo_btn': 8
        }

        if m.get_button_state(mapping['quit_demo_btn']):
            return True, None, None
        else:
            return False, None, None

    if m.update(0.0):
        # Drive mode
        if io_mode == MobileIOModes.DRIVE:
            return update_drive_mode(m)
        elif io_mode == MobileIOModes.DEPLOYED:
            return update_deployed_mode(m)
        elif io_mode == MobileIOModes.DEPLOYING:
            return update_deploying_mode(m)
    
    return False, None, None

def update_inputs(base_inputs: 'Optional[TreadyInputs]'=None, payload_inputs: 'Optional[CoreSamplerInputs]'=None):
    if base_inputs is None:
        base_inputs = TreadyInputs()
    if payload_inputs is None:
        payload_inputs = CoreSamplerInputs()
    
    base_inputs.deploy_safe = payload_control.sampler.base_deploy_safe
    base_inputs.stow_safe = payload_control.sampler.base_stow_safe
    base_inputs.payload_deployed = (payload_control.state == CoreSamplerControlState.DEPLOYED)

    payload_inputs.deploy_safe = base_control.base.payload_deploy_safe
    payload_inputs.stow_safe = base_control.base.payload_stow_safe
    payload_inputs.base_deployed = (base_control.state == TreadyControlState.DEPLOYED)

    return base_inputs, payload_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Treaded base family & names
    family = "Tready"
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # Payload family & names (fill family and names once known)
    payload_family = "Tready"
    payload_names = ["Mast_Pivot", "Chain_Upper", "Chain_Lower", "Wiggly-IO"]

    # mobileIO setup
    print('Looking for mobileIO device...')
    m = create_mobile_io(lookup, family)
    while m is None:
        print('Waiting for mobileIO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, family)
    
    print("mobileIO device found.")
    m.update()

    # Create base group
    base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)
    while base_group is None:
        print('Looking for Treadward modules...')
        sleep(1)
        base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)
    
    root_dir, _ = os.path.split(os.path.abspath(__file__))
    load_gains(base_group, os.path.join(root_dir, 'gains', 'smart-treadward-gains.xml'))

    base = TreadedBase(base_group, chassis_ramp_time=0.5, flipper_ramp_time=0.1)
    base.set_robot_model(os.path.join(root_dir, 'hrdf', 'Treadward.hrdf'))
    base_control = TreadyControl(base)

    # Create payload group
    payload_group = lookup.get_group_from_names(payload_family, payload_names)
    while payload_group is None:
        print('Looking for payload modules...')
        sleep(1)
        payload_group = lookup.get_group_from_names(payload_family, payload_names)
    
    root_dir, _ = os.path.split(os.path.abspath(__file__))
    load_gains(payload_group, os.path.join(root_dir, 'gains', 'core-sampling-gains.xml'))
    
    payload = CoreSampler(payload_group)
    #payload.set_robot_model(os.path.join(root_dir, 'hrdf', '')) # add payload hrdf once it exists
    payload_control = CoreSamplerControl(payload)

    # Update mobile io interface
    def update_mobile_io_base(controller: TreadyControl, new_state: TreadyControlState):
        global mobileIO_mode
        # Runs when base changes state
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

        elif new_state is TreadyControlState.DEPLOYING:
            controller.base.set_color('magenta')
            mobileIO_mode = MobileIOModes.DEPLOYING
            m.send_layout(layout_file=join(layout_dir, deploying_layout))
            msg = ('Robot Deploying\n'
                'Please wait...\n')
            set_mobile_io_instructions(m, msg, color="blue")

        elif new_state is TreadyControlState.TELEOP:
            controller.base.clear_color()
            mobileIO_mode = MobileIOModes.DRIVE
            m.send_layout(layout_file=join(layout_dir, drive_layout))
            m.set_axis_label(6, "BR") # TODO: this should not be needed but send layout has a bug
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
    
    def update_mobile_io_payload(controller: CoreSamplerControl, new_state: CoreSamplerControlState):
        global mobileIO_mode
        if controller.state == new_state:
                return
        
        elif new_state is CoreSamplerControlState.DEPLOYED:
            controller.sampler.clear_color()
            mobileIO_mode = MobileIOModes.DEPLOYED
            m.send_layout(layout_file=join(layout_dir, deployed_layout))
            m.set_axis_label(6, "Chain") # TODO: this should not be needed but send layout has a bug
            msg = ('Robot Ready to Control')
            set_mobile_io_instructions(m, msg, color="green")
        
        elif new_state is CoreSamplerControlState.STOWING:
            controller.sampler.set_color('magenta')
            mobileIO_mode = MobileIOModes.DEPLOYING
            m.send_layout(layout_file=join(layout_dir, deploying_layout))
            msg = ('Robot Stowing\n'
                'Please wait...')
            set_mobile_io_instructions(m, msg, color="blue")

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
    
    base_control._transition_handlers.append(update_mobile_io_base)
    base_control._update_handlers.append(update_torque_mode)

    payload_control._transition_handlers.append(update_mobile_io_payload)

    # can enable start logging here
    logging = False

    if logging:
        tready_dir = os.path.dirname(__file__)
        now = datetime.datetime.now()
        base.group.start_log(os.path.join(tready_dir, 'logs'), f'base_{now:%Y-%m-%d-%H:%M:%S}')

    while base_control.running and payload_control.running:
        t = time()
        try:
            quit, base_inputs, payload_inputs = parse_mobile_io_feedback(m, mobileIO_mode)
            if quit:
                break
            base_inputs, payload_inputs = update_inputs(base_inputs, payload_inputs)

            base_control.update(t, base_inputs)
            base_control.send()
            payload_control.update(t, payload_inputs)
            payload_control.send()
        except KeyboardInterrupt:
            break
    
    base_control.stop()

    if logging:
        base.group.stop_log()