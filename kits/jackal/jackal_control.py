
from multiprocessing.connection import Client
import struct
from time import time, sleep
from enum import Enum, auto
import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Optional
    from hebi._internal.mobile_io import MobileIO


class JackalControlState(Enum):
    STARTUP = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class JackalInputs:
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular


class JackalControl:
    def __init__(self, ip: str='localhost', port: int=6000, authkey: bytes=b'test'):
        self.state = JackalControlState.STARTUP
        self.conn = Client((ip, port), authkey=authkey)
        self.payload = struct.pack('<f', 0.0) + struct.pack('<f', 0.0)

    @property
    def running(self):
        return not self.state is self.state.EXIT

    def send(self):
        self.conn.send_bytes(self.payload)

    def update(self, t_now: float, base_input: 'Optional[JackalInputs]'):

        if self.state is self.state.EXIT:
            return False

        if not base_input:
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
                self.payload = struct.pack('<f', base_input.linear) + struct.pack('<f', base_input.angular)
                return True

            elif self.state is self.state.STARTUP:
                self.transition_to(self.state.TELEOP)
                return True

    def transition_to(self, state: JackalControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")
            self.payload = struct.pack('<f', 0.0) + struct.pack('<f', 0.0)

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        self.state = state


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None

    turn  = m.get_axis_state(1)
    move = m.get_axis_state(2)

    return JackalInputs(move, turn)


if __name__ == '__main__':
    lookup = hebi.Lookup()
    sleep(2)

    family = "Jackal"
    base_control = JackalControl()

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
    
    while base_control.running:
        t = time()
        inputs = parse_mobile_feedback(m)
        base_control.update(t, inputs)
        base_control.send()
