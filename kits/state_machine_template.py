
from time import time, sleep
from enum import Enum, auto
import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Optional, Callable
    from hebi._internal.mobile_io import MobileIO


class ControlState(Enum):
    STARTUP = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class Inputs:
    # This should probably be a data class, but Python 3.6 doesn't support those
    # Just a container for holding control values computed from Mobile IO in `parse_mobile_feedback`
    pass


class Control:
    def __init__(self):
        self.state = ControlState.STARTUP
        self._transition_handlers: 'list[Callable[[Control, ControlState], None]]' = []

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        pass

    def update(self, t_now: float, command_input: 'Optional[Inputs]'):

        if self.state is self.state.EXIT:
            return False

        if not command_input:
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
                # do something with input here
                return True

            elif self.state is self.state.STARTUP:
                self.transition_to(self.state.TELEOP)
                return True

    def transition_to(self, state: ControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        for handler in self._transition_handlers:
            handler(self, state)
        self.state = state


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None

    # Build an input object using the Mobile IO state
    return Inputs()


if __name__ == '__main__':
    lookup = hebi.Lookup()
    sleep(2)

    family = "HEBI"
    control = Control()

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, family)
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, family)

    #######################
    ## Main Control Loop ##
    #######################

    while control.running:
        t = time()
        inputs = parse_mobile_feedback(m)
        control.update(t, inputs)
        control.send()
