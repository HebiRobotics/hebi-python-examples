from time import sleep
import os

import hebi


def setup_base(lookup, base_family):
    from tready import TreadedBase
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # Create base group
    group = lookup.get_group_from_names([base_family], wheel_names + flipper_names)
    if group is None:
        raise RuntimeError(f"Could not find modules: {wheel_names + flipper_names} in family '{base_family}'")

    root_dir, _ = os.path.split(__file__)
    load_gains(group, os.path.join(root_dir, "gains/r-tready-gains.xml"))

    return TreadedBase(group, 0.25, 0.33)


def set_mobile_io_instructions(mobile_io, message, color=None):
    # Print Instructions
    mobile_io.clear_text()
    mobile_io.set_text(message)
    if color is not None:
        mobile_io.set_led_color(color)


def load_gains(group, gains_file):
    gains_command = hebi.GroupCommand(group.size)
    sleep(0.1)

    try:
        gains_command.read_gains(gains_file)
    except Exception as e:
        print(f'Warning - Could not load gains: {e}')
        return

    # Send gains multiple times
    for i in range(3):
        group.send_command(gains_command)
        sleep(0.1)


def setup_arm(lookup, family):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=os.path.join(root_dir, 'hrdf/tready-arm.hrdf'),
        lookup=lookup)

    mirror_group = lookup.get_group_from_names([family], ['J2B_shoulder1'])
    arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    arm.load_gains(os.path.join(root_dir, 'gains/tready-arm-gains.xml'))
    return arm


def setup_arm_6dof(lookup, family):
    root_dir, _ = os.path.split(__file__)
    # arm setup
    arm = hebi.arm.create(
        [family],
        ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
        hrdf_file=os.path.join(root_dir, 'hrdf/tready-arm-A2240-06G.hrdf'),
        lookup=lookup)

    arm.load_gains(os.path.join(root_dir, 'gains/A-2240-06.xml'))

    # Add the gripper
    gripper = hebi.arm.Gripper(lookup.get_group_from_names([family], ['gripperSpool']), -5, 1)
    gripper.load_gains(os.path.join(root_dir, 'gains/gripper_spool_gains.xml'))
    arm.set_end_effector(gripper)

    return arm
