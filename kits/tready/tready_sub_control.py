import hebi
from hebi.util import create_mobile_io
from time import time, sleep
import os
from .tready_utils import load_gains, set_mobile_io_instructions
from .tready import TreadedBase, TreadyControl, TreadyControlState, TreadyInputs, ChassisVelocity
from kits.arm.ex_leader_follower import LeaderFollowerControl, LeaderFollowerInputs
from kits.camera.pan_tilt_mast import MastControl, MastInputs, HebiCameraMast
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

    m.set_button_label(reset_pose_btn, '⟲')
    m.set_button_label(torque_btn, 'Torque')
    m.set_button_label(height_up_btn, '⤾◼⤿')
    m.set_button_label(recenter_btn, 'Center')
    m.set_button_label(height_down_btn, '⤿◼⤾')
    m.set_button_label(quit_demo_btn, '❌')

    m.set_button_mode(torque_btn, 1)

    m.set_axis_label(turn_joy, 'Turn')
    m.set_axis_label(forward_joy, 'Drive')
    m.set_axis_label(front_left_slider, 'FL')
    m.set_axis_label(front_right_slider, 'FR')
    m.set_axis_label(back_left_slider, 'BL')
    m.set_axis_label(back_right_slider, 'BR')

    for i in range(1, 9):
        m.set_snap(i, 0)
    
    m.set_led_color('yellow')

    # Arm
    follow_btn = 3
    haptic_btn = 4

    m.set_button_label(follow_btn, 'Follow')
    m.set_button_label(haptic_btn, 'Haptic')
    m.set_button_mode(follow_btn, 1)
    m.set_button_mode(haptic_btn, 1)
    m.set_button_output(follow_btn, 1)
    m.set_button_output(haptic_btn, 1)

    # Camera Mast
    pan_joy = 7
    tilt_joy = 8

    m.set_axis_label(pan_joy, 'Pan')
    m.set_axis_label(tilt_joy, 'Tilt')

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
                return True, None, None, None

            arm_inputs = LeaderFollowerInputs(
                follow=m.get_button_state(follow_btn) == 1,
                haptic_fbk=m.get_button_state(haptic_btn) == 1,
            )

            mast_inputs = MastInputs(
                [-m.get_axis_state(pan_joy), -m.get_axis_state(tilt_joy)],
                m.get_button_state(1)
            )

            if m.get_button_state(reset_pose_btn):
                return False, TreadyInputs(home=True, torque_mode=m.get_button_state(torque_btn), torque_toggle=abs(m.get_button_diff(torque_btn))), arm_inputs, mast_inputs
            
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
            
            return False, tready_inputs, arm_inputs, mast_inputs

        else:
            return False, None, None, None

    return parse_mobile_io_feedback


if __name__ == "__main__":
    root_dir, _ = os.path.split(os.path.abspath(__file__))
    lookup = hebi.Lookup()
    sleep(2)

    base_family = "Tready"
    arm_family = "Arm"
    flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
    wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

    # mobileIO setup
    print('Looking for mobileIO device...', end=' ')
    m = create_mobile_io(lookup, base_family)
    while m is None:
        try:
            print('Waiting for mobileIO device to come online...')
            sleep(1)
            m = create_mobile_io(lookup, base_family)
        except KeyboardInterrupt:
            exit(0)
    
    print("mobileIO device found.")
    m.update()
    parse_mobile_io_feedback = setup_mobile_io(m)

    # Create base group
    print('Looking for Tready modules...', end=' ')
    base_group = lookup.get_group_from_names(base_family, wheel_names + flipper_names)
    while base_group is None:
        try:
            print('Waiting for Tready modules...')
            sleep(1)
            base_group = lookup.get_group_from_names(base_family, wheel_names + flipper_names)
        except KeyboardInterrupt:
            exit(0)
    print('Tready modules found.')
    
    load_gains(base_group, os.path.join(root_dir, 'gains', 'smart-tready-gains.xml'))

    base = TreadedBase(base_group, chassis_ramp_time=0.33, flipper_ramp_time=0.1)
    base.set_robot_model(os.path.join(root_dir, 'hrdf', 'Tready.hrdf'))
    base_control = TreadyControl(base)
    base_control.namespace = "[Base] "

    # Leader Arm setup
    try:
        print('Looking for Leader Arm...', end=' ')
        leader_arm_family = "Tready-Sub-Leader"
        leader_module_names = [
            "J1_base",
            "J2_shoulder",
            "J3_elbow",
            "J4_wrist1",
            "J5_wrist2",
            "J6_wrist3",
        ]
        leader_hrdf_file = "../arm/config/hrdf/A-2085-06.hrdf"
        leader_gains_file = "../arm/config/gains/A-2085-06.xml"

        leader_hrdf_file = os.path.join(root_dir, leader_hrdf_file)
        leader_gains_file = os.path.join(root_dir, leader_gains_file)

        leader_arm = hebi.arm.create(
            [leader_arm_family],
            names=leader_module_names,
            hrdf_file=leader_hrdf_file,
            lookup=lookup,
        )
        leader_arm.load_gains(leader_gains_file)
        print('Leader Arm found.')
    except Exception as e:
        print(f'Failed to create Leader Arm: {e}')
        leader_arm = None

    # Follower Arm setup
    try:
        print('Looking for Follower Arm...', end=' ')
        follower_arm_family = "Tready-Sub-Follower"
        follower_module_names = [
            "J1_base",
            "J2_shoulder",
            "J3_elbow",
            "J4_wrist1",
            "J5_wrist2",
            "J6_wrist3",
        ]
        follower_hrdf_file = "../arm/config/hrdf/A-2085-06.hrdf"
        follower_gains_file = "../arm/config/gains/A-2085-06.xml"

        follower_hrdf_file = os.path.join(root_dir, follower_hrdf_file)
        follower_gains_file = os.path.join(root_dir, follower_gains_file)

        follower_arm = hebi.arm.create(
            [follower_arm_family],
            names=follower_module_names,
            hrdf_file=follower_hrdf_file,
            lookup=lookup,
        )
        follower_arm.load_gains(follower_gains_file)
        print('Follower Arm found.')

    except Exception as e:
        print(f'Failed to create Follower Arm: {e}')
        leader_follower_control = None

    # Setup LeaderFollowerControl
    if leader_arm is not None and follower_arm is not None:
        leader_follower_control = LeaderFollowerControl(
            leader_arm, follower_arm, home_pose=[0.0, 2.09, 2.09, 0.0, 1.57, 0.0]
        )
        leader_follower_control.namespace = "[Arm] "

    # Create camera mast group
    try:
        print('Looking for Camera Mast...', end=' ')
        mast_family = "Tready-Sub"
        mast_module_names = ['J1_pan', 'J2_tilt']
        mast_group = lookup.get_group_from_names(mast_family, mast_module_names)
        if mast_group is None:
            raise RuntimeError('Could not find mast group. Check that family and names match actuators on the network.')
        else:
            mast = HebiCameraMast(mast_group)
            mast_control = MastControl(mast)
            mast_control.namespace = "[Mast] "
        print('Camera Mast found.')
    except Exception as e:
        print(f'Failed to create Camera Mast: {e}')
        mast = None

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
            controller.base.clear_color()
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

    print("\n")

    # can enable start logging here
    while base_control.running and (leader_arm is None or follower_arm is None or leader_follower_control.running) and (mast is None or mast_control.running):
        t = time()
        try:
            quit, base_inputs, arm_inputs, mast_inputs = parse_mobile_io_feedback(m)
            if quit:
                break
            base_control.update(t, base_inputs)
            if leader_arm is not None and follower_arm is not None:
                leader_follower_control.update(t, arm_inputs)
            if mast is not None:
                mast_control.update(t, mast_inputs)
            
            base_control.send()
            if leader_arm is not None and follower_arm is not None:
                leader_follower_control.send()
            if mast is not None:
                mast_control.send()
        except KeyboardInterrupt:
            break
    
    base_control.stop()
    if leader_arm is not None and follower_arm is not None:
        leader_follower_control.stop()
    if mast is not None:
        mast_control.stop()
    # stop logging here