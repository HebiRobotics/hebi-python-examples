from time import sleep

import hebi
from hebi.util import create_mobile_io

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO


def wait_for_mobile_io(lookup, family):
    # mobileIO setup
    print("Looking for mobileIO device...")
    m = create_mobile_io(lookup, family)
    count = 0
    while m is None:
        print("Waiting for mobileIO device to come online...")
        if count > 10:
            count = 0
            lookup.reset()
        sleep(1)
        count += 1
        m = create_mobile_io(lookup, family)

    return m


def set_mobile_io_instructions(mobile_io: "MobileIO", message, color=None):
    # Print Instructions
    if color is not None:
        mobile_io.set_led_color(color, blocking=False)
    mobile_io.clear_text(blocking=False)
    mobile_io.add_text(message, blocking=False)
    print("[mobileIO] " + message)


def load_gains(group: "Group", gains_file: str):
    gains_command = hebi.GroupCommand(group.size)
    sleep(0.1)

    try:
        gains_command.read_gains(gains_file)
    except Exception as e:
        print(f"Warning - Could not load gains: {e}")
        return

    # Send gains multiple times
    for _ in range(3):
        group.send_command(gains_command)
        sleep(0.1)
