#! /usr/bin/env python3

from time import sleep
from os import path
import subprocess

import hebi
from hebi.util import create_mobile_io

DEMOS = {
    'Tready No Arm': 'tready.py',
    'Tready Joystick Arm': 'tready_arm_joystick_control.py',
    'Tready MobileIO Arm': 'tready_arm_io_control.py',
}

def launch_demo(demo):
    if demo not in DEMOS.keys():
        print(f"ERROR: No demo #{demo} set, cannot launch!")
        return

    root = path.split(path.abspath(__file__))[0]
    demo_file = path.join(root, DEMOS[demo])
    print(f'Launching {DEMOS[demo]}')
    
    subprocess.check_output(['python3', demo_file])

def select_demo(mobile_io):
    for i, k in enumerate(DEMOS.keys()):
        if m.get_button_state(i+1):
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
        m.clear_text()
        demos_text = [f'B{i+1}: {k}\n' for i, k in enumerate(DEMOS.keys())]
        m.set_text(''.join(demos_text))
        # clear buttons before demo selection
        m.set_led_color('white')
        for i in range(8):
            m.set_button_mode(i+1, 0)
            m.set_button_output(i+1, 0)

        #######################
        ## Demo Select Loop ##
        #######################
        while True:
            if m.update():
                if next_demo := select_demo(m):
                    # delete the mobileIO group so it doesn't interfere w/ demo
                    del m # does this actually delete the group?
                    launch_demo(next_demo) # blocking
                    break # break to outer loop when demo done
        print('here?')
                
    sys.exit(0)
