import sys
import os
from time import time, sleep
from .oldtready import TreadyControl, TreadyControlState, config_mobile_io

from advanced.demos.MAPS_control.gripper_control import GripperControl
from advanced.demos.MAPS_control.MAPS_input_device_example import ContinuousAngleMaps, LeaderFollowerControl, LeaderFollowerControlState
from advanced.demos.MAPS_control.MAPS_input_device_w_gripper_example import setup_mobile_io, parse_mobile_feedback

import numpy as np

import hebi
from hebi.util import create_mobile_io
from hebi.arm import Gripper


if __name__ == "__main__":
    from .tready_utils import set_mobile_io_instructions, setup_base

    lookup = hebi.Lookup()
    sleep(2)

    family = "Tready"

    # Base setup
    base = setup_base(lookup, family)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for mobileIO device to come online...')
    m_tready = create_mobile_io(lookup, family, phone_name)
    while m_tready is None:
        m_tready = create_mobile_io(lookup, family, phone_name)
        print("Could not find mobileIO device, waiting...")
        sleep(0.5)

    print("mobileIO device found.")
    input_parser = config_mobile_io(m_tready)

    demo_controller = TreadyControl(base)

    def update_mobile_io(controller: TreadyControl, new_state: TreadyControlState):
        if controller.state == new_state:
            return

        if new_state is controller.state.HOMING:
            controller.base.set_color('magenta')
            msg = ('Robot Homing Sequence\n'
                   'Please wait...')
            set_mobile_io_instructions(m_tready, msg, color="blue")

        elif new_state is controller.state.TELEOP:
            controller.base.clear_color()
            msg = ('Robot Ready to Control\n'
                   'B1: Reset\n'
                   'B6: Joined Flipper\n'
                   'B8 - Quit')
            set_mobile_io_instructions(m_tready, msg, color="green")

        elif new_state is controller.state.DISCONNECTED:
            controller.base.set_color('blue')

        elif new_state is controller.state.EXIT:
            controller.base.set_color("red")

            # unset mobileIO control config
            m_tready.set_button_mode(6, 0)
            m_tready.set_button_output(1, 0)
            m_tready.set_button_output(8, 0)
            set_mobile_io_instructions(m_tready, 'Demo Stopped', color="red")

    demo_controller._transition_handlers.append(update_mobile_io)

    # Setup MAPS leader/follower
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, 'Arm')
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, 'Arm')

    setup_mobile_io(m)

    maps_modules = ['J1-A', 'J2-B', 'J2-A', 'J3-B', 'J3-A', 'J4-B', 'J4-A']
    maps_group = lookup.get_group_from_names(['MAPS'], maps_modules)
    while maps_group is None:
        m.clear_text()
        m.add_text('MAPS arm not found: Check connection and make sure all modules are blinking green')
        sleep(1)
        maps_group = lookup.get_group_from_names(['MAPS'], maps_modules)

    # need these b/c MAPS joint zeros are in different locations
    angle_offsets = np.array([0.0, np.pi / 2, -np.pi, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
    input_arm = ContinuousAngleMaps(maps_group, angle_offsets)

    base_dir, _ = os.path.split(__file__)
    base_dir = os.path.join(base_dir, '../arm')

    output_arm = hebi.arm.create(
        ['Arm'],
        ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2'],
        hrdf_file=os.path.join(base_dir, 'hrdf/A-2303-01.hrdf'),
        lookup=lookup)

    mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    while mirror_group is None:
        m.clear_text()
        m.add_text('Still looking for mirror group...')
        sleep(1)
        mirror_group = lookup.get_group_from_names(['Arm'], ['J2B_shoulder1'])
    # mirror the position/velocity/effort of module 1 ('J2A_shoulder1') to the module
    # in the mirror group ('J2B_shoulder1')
    # Keeps the two modules in the double shoulder bracket in sync
    output_arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))

    output_arm.load_gains(os.path.join(base_dir, 'gains/A-2303-01.xml'))
    # need to update the gains for the mirror group also
    gains_cmd = hebi.GroupCommand(1)
    gains_cmd.read_gains(os.path.join(base_dir, 'gains/mirror_shoulder.xml'))
    mirror_group.send_command_with_acknowledgement(gains_cmd)

    output_arm.cancel_goal()

    gripper_group = lookup.get_group_from_names(['Arm'], ['gripperSpool'])
    while gripper_group is None:
        m.clear_text()
        m.add_text("Looking for gripper module 'Arm/gripperSpool' ...")
        sleep(1)
        gripper_group = lookup.get_group_from_names(['Arm'], ['gripperSpool'])

    gripper = Gripper(gripper_group, -5, 1)
    gripper.load_gains(os.path.join(base_dir, 'gains/gripper_spool_gains.xml'))

    input_arm.update()
    output_arm.update()

    output_joints_home = [0.0, -1.0, 0.0, -0.6, -np.pi / 2, 1.0, 0.0]
    # allowed angular difference (°) per joint before starting align
    allowed_diff = np.array([30.0, 20.0, 30.0, 20.0, 45.0, 45.0, 360.0])

    leader_follower_control = LeaderFollowerControl(input_arm, output_arm, output_joints_home, allowed_diff)
    gripper_control = GripperControl(gripper)

    # Update text/border color on state transitions

    def update_ui(controller: LeaderFollowerControl, new_state: LeaderFollowerControlState):
        if controller.state == new_state:
            return

        if new_state == LeaderFollowerControlState.HOMING:
            m.set_led_color('blue', blocking=False)
            m.clear_text()
            m.add_text("Homing...")
        elif new_state == LeaderFollowerControlState.UNALIGNED:
            m.set_led_color('blue', blocking=False)
        elif new_state == LeaderFollowerControlState.ALIGNED:
            m.set_led_color('green', blocking=False)
            m.clear_text()
            m.add_text("Aligned!")

    leader_follower_control._transition_handlers.append(update_ui)

    #######################
    ## Main Control Loop ##
    #######################

    #demo_controller.start_logging()
    #last_log_start_time = time()
    last_text_update = 0.0
    while demo_controller.running and gripper_control.running and leader_follower_control.running:
        t = time()
        demo_inputs = None
        if m_tready.update(0.0):
            demo_inputs = input_parser(m_tready)

        arm_inputs, gripper_inputs = parse_mobile_feedback(m)

        try:
            demo_controller.update(t, demo_inputs)
            leader_follower_control.update(t, arm_inputs)
            gripper_control.update(t, gripper_inputs)

            demo_controller.send()
            leader_follower_control.send()
            gripper_control.send()

            if leader_follower_control.state == LeaderFollowerControlState.UNALIGNED:
                if t - last_text_update > 0.1:
                    last_text_update = t
                    m.clear_text()
                    m.add_text(f'Unaligned: {np.around(np.rad2deg(leader_follower_control.angle_diff), decimals=0)}')

        except KeyboardInterrupt:
            demo_controller.transition_to(t, TreadyControlState.EXIT)

        if m_tready.get_button_state(8):
            demo_controller.transition_to(t, TreadyControlState.EXIT)

        #if t - last_log_start_time > 3600:
        #    demo_controller.cycle_log()
        #    last_log_start_time = t

    sys.exit(0)
