#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass

from pathlib import Path
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io
from hebi._internal.group_manager import GroupManager

from .camera import HebiCamera

import typing
from typing import override

if typing.TYPE_CHECKING:
    from typing import Sequence
    from hebi._internal.mobile_io import MobileIO
    from hebi._internal.group import Group
    from hebi.robot_model import RobotModel
    from typing import Self, Callable


class CameraMast(GroupManager):
    def __init__(
        self,
        pan_tilt_group: Group,
        hrdf: RobotModel | str = str(Path(__file__).parent / 'config' / 'hrdf' / 'pan_tilt_mast.hrdf'),
    ):
        super().__init__(time.time, pan_tilt_group)

        if isinstance(hrdf, str):
            self.robot_model = hebi.robot_model.import_from_hrdf(hrdf)
        else:
            self.robot_model = hrdf

        self.output_frame = np.empty((4, 4), dtype=np.float64)

    def _inner_update(self, t: float) -> bool:
        res = super()._inner_update(t)

        base_frame = np.eye(4)
        base_frame[:3, :3] = R.from_quat(self.last_feedback[0].orientation, scalar_first=True).as_matrix()
        self.robot_model.base_frame = base_frame
        self.robot_model.get_end_effector(self.last_feedback.position, output=self.output_frame)
        return res

    def set_velocity(self, duration: float, velocity: Sequence[float]):
        g = hebi.Goal(2).add_waypoint(duration, np.full(2, np.nan), velocity)
        self.set_goal(g)

    def set_position(self, duration: float, position: Sequence[float]):
        g = hebi.Goal(2).add_waypoint(duration, position)
        self.set_goal(g)


@dataclass
class MastInputs:
    v_pan: float
    v_tilt: float
    home: bool
    zoom: float
    light_enable: bool
    light_pwm: float


@dataclass
class ControllerState:
    controller: MastController

    def can_transition_to(self, state: ControllerState) -> bool:
        return True

    def can_transition_from(self, state: ControllerState) -> bool:
        return True

    def on_enter(self, from_state: ControllerState, **kwargs): ...

    def update(self, **kwargs): ...

    def receive_inputs(self, t_now: float, inputs, **kwargs): ...

    def on_exit(self, to_state: ControllerState, **kwargs): ...


@dataclass
class CameraMastState(ControllerState):
    controller: MastController

    @override
    def receive_inputs(self, t_now: float, inputs: MastInputs, **kwargs): ...


@dataclass
class CameraMastStartupState(ControllerState):
    def update(self, **kwargs):
        self.controller.transition_to(CameraMastHomingState(self.controller))


@dataclass
class CameraMastHomingState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.mast.set_position(3.0, [np.pi, np.pi / 2])

    def update(self, **kwargs):
        if self.controller.mast.at_goal:
            self.controller.transition_to(CameraMastTeleopState(self.controller))


@dataclass
class CameraMastTeleopState(ControllerState):
    PAN_SCALING = 0.5
    TILT_SCALING = 0.5

    def receive_inputs(self, t_now: float, inputs: MastInputs, **kwargs):
        self.controller.camera.zoom_level = inputs.zoom
        self.controller.camera.light_enable = inputs.light_enable
        self.controller.camera.light_pwm = inputs.light_pwm

        if inputs.home:
            self.controller.transition_to(CameraMastHomingState(self.controller))
        else:
            Δpan = inputs.v_pan * self.PAN_SCALING
            Δtilt = inputs.v_tilt * self.TILT_SCALING
            Δt = 0.25  # sec

            self.controller.mast.set_velocity(Δt, [Δpan, Δtilt])


@dataclass
class CameraMastDisconnectedState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.mast.cancel_goal()

    def update(self, **kwargs):
        if (time.time() - self.controller.last_cmd_t) < 1.0:
            self.controller.transition_to(CameraMastTeleopState(self.controller))


@dataclass
class CameraMastExitState(ControllerState): ...


class MastController:
    def __init__(
        self,
        mast: CameraMast,
        camera: HebiCamera,
        starting_state: type[ControllerState],
        fault_state: type[ControllerState],
        exit_state: type[ControllerState],
    ):

        self.mast = mast
        self.camera = camera
        self.state = starting_state(self)
        self.fault_state = fault_state
        self.exit_state = exit_state

        self.namespace = ""
        # self._executer = futures.ThreadPoolExecutor(max_workers=1)

        self.last_cmd_t = time.time()

        self.payload_deploy_safe = False
        self.payload_stow_safe = True

        self._transition_handlers: list[Callable[[Self, ControllerState], None]] = []
        self._update_handlers: list[Callable[[Self, ControllerState], None]] = []

        self.mast.set_on_update_callback(self.on_mast_update)
        self.mast.set_on_failed_update_callback(self.on_packet_loss)

        if not self.mast.running:
            self.mast.start()

        if not self.camera.running:
            self.camera.start()

    @property
    def running(self):
        return not isinstance(self.state, self.exit_state)

    def receive_inputs(self, input: MastInputs):
        t_now = time.time()
        self.last_cmd_t = t_now
        self.state.receive_inputs(t_now, input)

    def transition_to(self, new_state: ControllerState, **kwargs):
        if not new_state.can_transition_from(self.state):
            return False
        elif not self.state.can_transition_to(new_state):
            return False
        elif isinstance(self.state, type(new_state)):
            return True

        print(f'{self.namespace} Transition {type(self.state)} -> {type(new_state)}')
        self.state.on_exit(to_state=new_state, **kwargs)
        new_state.on_enter(from_state=self.state, **kwargs)
        self.state = new_state

        return True

    def clear_leds(self):
        self.mast.pending_command.led.color = 'transparent'

    def wait_for_shutdown(self, timeout_s):
        self.mast.stop()
        self.camera.stop()
        time.sleep(timeout_s)

        if self.mast.running:
            print("Warning: base did not shutdown cleanly")

    def start_logging(self):
        self.mast.group.start_log("logs", mkdirs=True)

    def cycle_log(self):
        self.mast.group.stop_log()
        self.mast.group.start_log("logs", mkdirs=True)

    def on_packet_loss(self):
        print('WARNING: Failed to get next feedback!!!')

    def on_mast_update(self):
        if isinstance(self.state, self.exit_state):
            return True

        if self.mast.m_stopped:
            if not isinstance(self.state, self.fault_state):
                self.transition_to(self.fault_state(self))
            return True

        # Transition to disconnected if no mobile update recieved
        if not isinstance(self.state, CameraMastDisconnectedState):
            if (time.time() - self.last_cmd_t) > 1.0:
                self.transition_to(CameraMastDisconnectedState(self))
                return True

        self.state.update()
        return True


def parse_mobile_feedback(m: MobileIO):
    if not m.update(0.0):
        return None

    # Update Camera zoom/lights
    zoom = 1 + (m.get_axis_state(3) + 1.0) / 2.0
    light = (m.get_axis_state(4) + 1.0) / 2.0
    light_en = m.get_button_state(2)

    pan = -1.0 * m.get_axis_state(1)
    tilt = m.get_axis_state(2)

    return MastInputs(pan, tilt, m.get_button_state(1), zoom, light_en, light)


if __name__ == "__main__":
    lookup = hebi.Lookup()

    # Setup Camera Pan/Tilt
    family = "Mast"
    module_names = ['J1_pan', 'J2_tilt']

    group = lookup.get_group_from_names(family, module_names, timeout_ms=2000)
    while group is None:
        print('Looking for pan/tilt modules...')
        group = lookup.get_group_from_names(family, module_names, timeout_ms=1000)

    mast = CameraMast(group)
    camera = HebiCamera.from_family('HEBI')
    camera.set_overlay_text('HEBI Mast Control Demo')

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, family)
    while m is None:
        try:
            print('Waiting for Mobile IO device to come online...')
            time.sleep(1)
            m = create_mobile_io(lookup, family)
        except KeyboardInterrupt:
            exit(0)

    m.update()
    m.send_layout(layout_file=str(Path(__file__).parent / 'config' / 'layouts' / 'CameraMastControl.json'))

    mast_control = MastController(mast, camera, CameraMastStartupState, CameraMastDisconnectedState, CameraMastExitState)

    #######################
    ## Main Control Loop ##
    #######################

    try:
        while mast_control.running:
            t = time.time()
            inputs = parse_mobile_feedback(m)
            if inputs:
                if m.get_button_state(8):
                    break
                mast_control.receive_inputs(inputs)
    except KeyboardInterrupt:
        pass

    print('Quitting')
    camera.clear_overlay_text()
    mast_control.wait_for_shutdown(2)
