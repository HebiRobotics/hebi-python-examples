from time import sleep
import os

import hebi

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO
    from hebi import Lookup


def setup_base(lookup: 'Lookup', base_family: str):
    from .tready import TreadedBase
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # Create base group
    group = lookup.get_group_from_names([base_family], wheel_names + flipper_names)
    if group is None:
        raise RuntimeError(f"Could not find modules: {wheel_names + flipper_names} in family '{base_family}'")

    root_dir, _ = os.path.split(__file__)
    load_gains(group, os.path.join(root_dir, "gains/r-tready-gains.xml"))

    return TreadedBase(group, 0.25, 0.33)


def set_mobile_io_instructions(mobile_io: 'MobileIO', message, color=None):
    # Print Instructions
    if color is not None:
        mobile_io.set_led_color(color, blocking=False)
    mobile_io.clear_text(blocking=False)
    mobile_io.add_text(message, blocking=False)
    print(message)


def load_gains(group: 'Group', gains_file: str):
    gains_command = hebi.GroupCommand(group.size)
    sleep(0.1)

    try:
        gains_command.read_gains(gains_file)
    except Exception as e:
        print(f'Warning - Could not load gains: {e}')
        return

    # Send gains multiple times
    for _ in range(3):
        group.send_command(gains_command)
        sleep(0.1)


def setup_arm(lookup: 'Lookup', family: str):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=os.path.join(root_dir, 'hrdf/tready-arm.hrdf'),
        lookup=lookup)

    mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder1'])
    while mirror_group is None:
        print(f'Looking for double shoulder module "{family}/J2B_shoulder1"...')
        sleep(1)
        mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder1'])

    arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    arm.load_gains(os.path.join(root_dir, 'gains/tready-arm-gains.xml'))
    return arm


def setup_arm_7dof(lookup: 'Lookup', family: str):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2'],
        hrdf_file=os.path.join(root_dir, 'hrdf/tready-7dof-arm.hrdf'),
        lookup=lookup)

    mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder1'])
    while mirror_group is None:
        print(f'Looking for double shoulder module "{family}/J2B_shoulder1"...')
        sleep(1)
        mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder1'])

    arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    arm.load_gains(os.path.join(root_dir, 'gains/tready-7dof-arm-gains.xml'))

    # Add the gripper
    gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])
    while gripper_group is None:
        print(f'Looking for gripper module "{family}/gripperSpool"...')
        sleep(1)
        gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])

    gripper = hebi.arm.Gripper(gripper_group, -5, 1)
    gripper.load_gains(os.path.join(root_dir, 'gains/gripper_spool_gains.xml'))
    arm.set_end_effector(gripper)

    return arm


def setup_arm_6dof(lookup: 'Lookup', family: str, with_gripper: bool = True):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=os.path.join(root_dir, 'hrdf/tready-arm-A2240-06G.hrdf'),
        lookup=lookup)

    arm.load_gains(os.path.join(root_dir, 'gains/A-2240-06.xml'))

    if with_gripper:
        # Add the gripper
        gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])
        while gripper_group is None:
            print(f'Looking for gripper module "{family}/gripperSpool"...')
            sleep(1)
            gripper_group = lookup.get_group_from_names([family], ['gripperSpool'])

        gripper = hebi.arm.Gripper(gripper_group, -5, 1)
        gripper.load_gains(os.path.join(root_dir, 'gains/gripper_spool_gains.xml'))
        arm.set_end_effector(gripper)

    return arm
