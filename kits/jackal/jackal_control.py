
from multiprocessing.connection import Client
import struct
from time import time, sleep
from enum import Enum, auto
import hebi
from hebi.util import create_mobile_io
import numpy as np

import typing
if typing.TYPE_CHECKING:
    from typing import Optional, Callable
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
        self.linear = 0.0
        self.angular = 0.0
        self.payload = struct.pack('<f', self.linear) + struct.pack('<f', self.angular)
        self._transition_handlers: 'list[Callable[[JackalControl, JackalControlState], None]]' = []
        self.trajectory = hebi.trajectory.create_trajectory([0, 0.5], np.zeros((2, 2)))
        self.last_input_time = 0.0

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.payload = struct.pack('<f', self.linear) + struct.pack('<f', self.angular)
        self.conn.send_bytes(self.payload)

    def update(self, t_now: float, base_input: 'Optional[JackalInputs]'):

        if self.state is self.state.EXIT:
            return False

        if not base_input:
            if t_now - self.last_input_time > 1.0 and self.state is not self.state.DISCONNECTED:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return True
        else:
            self.last_input_time = t_now

            if self.state is self.state.DISCONNECTED:
                self.last_input_time = t_now
                self.transition_to(t_now, self.state.TELEOP)
                return True

            elif self.state is self.state.TELEOP:
                _, v, a = self.trajectory.get_state(t_now)
                self.linear = v[0]
                self.angular = v[1]

                # replan at 10 Hz
                if self.trajectory.start_time + 0.1 < t_now:
                    positions = np.zeros((2, 2))
                    positions[:, 1] = np.nan

                    velocities = np.empty((2, 2))
                    velocities[0, 0] = self.linear
                    velocities[0, 1] = base_input.linear

                    velocities[1, 0] = self.angular
                    velocities[1, 1] = base_input.angular

                    accelerations = np.zeros((2, 2))
                    accelerations[:, 0] = a

                    self.trajectory = hebi.trajectory.create_trajectory([t_now, t_now+0.5], positions, velocities, accelerations)

                return True

            elif self.state is self.state.STARTUP:
                self.transition_to(t_now, self.state.TELEOP)
                return True

    def transition_to(self, t_now: float, state: JackalControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")
            self.linear = 0.0
            self.angular = 0.0

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        for handler in self._transition_handlers:
            handler(self, state)
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
