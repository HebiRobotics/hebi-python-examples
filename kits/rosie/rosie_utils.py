from time import sleep
import os

import hebi

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO
    from hebi import Lookup
    from hebi.arm import Arm, Gripper
    from hebi.config import HebiConfig


def setup_base(lookup: 'Lookup', base_family: str):
    from ..bases.omni_base import OmniBase
    wheel_names = ['W1', 'W2', 'W3']

    # Create base group
    group = lookup.get_group_from_names([base_family], wheel_names)
    if group is None:
        raise RuntimeError(f"Could not find modules: {wheel_names} in family '{base_family}'")

    #root_dir, _ = os.path.split(__file__)
    #load_gains(group, os.path.join(root_dir, "gains/rosie-wheel-gains.xml"))

    return OmniBase(group)


def set_mobile_io_instructions(mobile_io: 'MobileIO', message, color=None):
    # Print Instructions
    if color is not None:
        mobile_io.set_led_color(color, blocking=False)
    mobile_io.clear_text(blocking=False)
    mobile_io.add_text(message, blocking=False)
    print(message)


def setup_arm(cfg: 'HebiConfig', lookup: 'Lookup'):
    gripper = None
    # arm setup
    try:
        arm = hebi.arm.create_from_config(cfg, lookup=lookup)
    except Exception as e:
        print(e)
        arm = None
    
    has_gripper = False
    if user_data := cfg.user_data:
        has_gripper = user_data['has_gripper']

    if arm and has_gripper:
        family = cfg.families[0]
        # Add the gripper
        gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])
        tries = 3
        while gripper_group is None and tries > 0:
            print(f'Looking for gripper module "{family}/gripperSpool"...')
            sleep(1)
            gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])
            tries -= 1
        
        if gripper_group is None:
            return arm, gripper

        gripper_close_effort = -5
        gripper_open_effort = 1
        if user_data := cfg.user_data:
            gripper_close_effort = user_data['gripper_open_effort']
            gripper_open_effort = user_data['gripper_close_effort']

        gripper = hebi.arm.Gripper(gripper_group, gripper_close_effort, gripper_open_effort)
        if cfg.gains is not None:
            gripper_gains_file = cfg.gains['gripper']
            gripper.load_gains(os.path.join(cfg.config_location, gripper_gains_file))
        gripper.open()

    return arm, gripper
