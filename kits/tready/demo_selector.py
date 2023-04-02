#! /usr/bin/env python3

from time import sleep
import os
import subprocess

import hebi
from hebi.util import create_mobile_io

#'Tready MAPS Control': 'kits.tready.tready_leader_follower_control',

DEMOS = {
    'Leader-Follower Ctrl': 'advanced.demos.MAPS_control.MAPS_input_device_6DoF_example',
    'Tready Joystick Cam': 'kits.tready.tready_arm_joystick_control_6DoF_camera',
}


def launch_demo(demo):
    if demo not in DEMOS.keys():
        print(f"ERROR: No demo #{demo} set, cannot launch!")
        return

    print(f'Launching {DEMOS[demo]}')
    root = os.path.split(os.path.abspath(__file__))[0]
    kits_dir = root.split('kits')[0]
    os.chdir(kits_dir)
    subprocess.check_output(['python3', '-m', f'{DEMOS[demo]}'])


def select_demo(mobile_io):
    for i, k in enumerate(DEMOS.keys()):
        if m.get_button_state(i + 1):
            return k
    else:
        return None


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # mobileIO setup
    family = "Tready"
    phone_name = "mobileIO"

    # Outer loop, create mobileIO, set text
    while True:
        print('Waiting for mobileIO device to come online...')
        m = create_mobile_io(lookup, family, phone_name)
        while m is None:
            m = create_mobile_io(lookup, family, phone_name)
            print("Could not find mobileIO device, waiting...")
            sleep(0.5)

        print("mobileIO device found.")
        # clear buttons before demo selection
        m.resetUI()
        demos_text = [f'B{i+1}: {k}\n' for i, k in enumerate(DEMOS.keys())]
        m.add_text(''.join(demos_text))

        #######################
        ## Demo Select Loop ##
        #######################
        while True:
            if m.update():
                next_demo = select_demo(m)
                if next_demo is not None:
                    # delete the mobileIO group so it doesn't interfere w/ demo
                    del m  # does this actually delete the group?
                    launch_demo(next_demo)  # blocking
                    break  # break to outer loop when demo done
        print('here?')

    sys.exit(0)
