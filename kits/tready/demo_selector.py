#! /usr/bin/env python3

from time import sleep, time
import os
import subprocess

import hebi
from hebi.util import create_mobile_io

#'Tready MAPS Control': 'kits.tready.tready_leader_follower_control',

DEMOS = {
    'No Arm': 'kits.tready.tready',
    'MAPS': 'advanced.demos.MAPS_control.MAPS_input_device_6DoF_example',
    'Joy': 'kits.tready.tready_arm_joystick_control',
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


def select_demo(m):
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

    last_seen_time = 0.0

    # Outer loop, create mobileIO, set text
    while True:
        print('Waiting for mobileIO device to come online...')
        m = create_mobile_io(lookup, family, phone_name)
        while m is None:
            m = create_mobile_io(lookup, family, phone_name)
            print("Could not find mobileIO device, waiting...")
            sleep(0.5)

        print("mobileIO device found.")
        last_seen_time = time()
        # clear buttons before demo selection
        m.resetUI()
        for i in range(8):
            m.set_snap(i+1, None)

        demos_text = [f'B{i+1}: {k}\n' for i, k in enumerate(DEMOS.keys())]
        for i, k in enumerate(DEMOS.keys()):
            m.set_button_label(i+1, k)

        #######################
        ## Demo Select Loop ##
        #######################
        while True:
            if m.update(0.0):
                last_seen_time = time()
                next_demo = select_demo(m)
                if next_demo is not None:
                    # delete the mobileIO group so it doesn't interfere w/ demo
                    del m  # does this actually delete the group?
                    launch_demo(next_demo)  # blocking
                    break  # break to outer loop when demo done
            else:
                if time() - last_seen_time > 2:
                    lookup.reset()
                    break
        print('here?')

    sys.exit(0)
