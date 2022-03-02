from time import time, sleep
from enum import Enum, auto
import hebi
from hebi.util import create_mobile_io
from hebi.arm import Gripper

import typing
if typing.TYPE_CHECKING:
    from typing import Optional
    from hebi._internal.mobile_io import MobileIO


class GripperControlState(Enum):
    STARTUP = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class GripperInputs:
    def __init__(self, gripper_target):
        self.gripper_target = gripper_target


class GripperControl:
    def __init__(self, gripper: 'Gripper'):
        self.state = GripperControlState.STARTUP
        self.gripper = gripper
        self.last_input_time = time()

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.gripper.send()

    def update(self, t_now: float, gripper_input: 'Optional[GripperInputs]'):

        if self.state is self.state.EXIT:
            return False

        if not gripper_input:
            if t_now - self.last_input_time > 1.0 and self.state is not self.state.DISCONNECTED:
                print("mobileIO timeout, disabling motion")
                self.transition_to(self.state.DISCONNECTED)
            return True
        else:
            self.last_input_time = t_now

            if self.state is self.state.DISCONNECTED:
                self.last_input_time = t_now
                self.transition_to(self.state.TELEOP)
                return True

            elif self.state is self.state.TELEOP:
                self.gripper.update(gripper_input.gripper_target)
                # do something with input here
                return True

            elif self.state is self.state.STARTUP:
                self.transition_to(self.state.TELEOP)
                return True

    def transition_to(self, state: GripperControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        self.state = state


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None

    if m.get_button_diff(2) == 1:
        gripper_target = 1.0
        m.set_axis_value(3, 1.0)
    elif m.get_button_diff(4) == 1:
        gripper_target = 0.0
        m.set_axis_value(3, -1.0)
    else:
        # rescale to range [0, 1]
        gripper_target = (m.get_axis_state(3) + 1.0) / 2.0

    # Build an input object using the Mobile IO state
    return GripperInputs(gripper_target)


def setup_mobile_io(m: 'MobileIO'):
    m.set_button_label(2, 'close')
    m.set_button_label(4, 'open')
    m.set_axis_label(3, 'grip')
    m.set_axis_value(3, -1.0)


if __name__ == '__main__':
    lookup = hebi.Lookup()
    sleep(2)

    # Create the gripper 
    family = 'Arm'
    name   = 'gripperSpool'

    gripper_group = lookup.get_group_from_names([family], [name])
    while gripper_group is None:
        print(f'Looking for gripper module {family} / {name} ...')
        sleep(1)
        gripper_group = lookup.get_group_from_names([family], [name])

    gripper = Gripper(gripper_group, -5, 1)
    gripper.load_gains('arm/gains/gripper_spool_gains.xml')

    control = GripperControl(gripper)

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, family)
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, family)

    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    while control.running:
        t = time()
        try:
            inputs = parse_mobile_feedback(m)
            control.update(t, inputs)
            control.send()
        except KeyboardInterrupt:
            control.transition_to(GripperControlState.EXIT)
