#!/usr/bin/env python3
"""Pan/tilt camera mast driven from a Mobile IO.

    python -m kits.camera.pan_tilt_mast pan_tilt_mast.cfg.yaml

The config gives the mast modules and hrdf, and ``camera_family`` in user_data.
"""

from __future__ import annotations

import sys
import time
import typing
from dataclasses import dataclass, field
from pathlib import Path

import hebi
import numpy as np
from hebi.arm import GroupManager
from hebi.util import create_mobile_io
from scipy.spatial.transform import Rotation as R

from kits.common.config import add_config_argument, config_arg_parser, load_config, required_user_data
from kits.common.state_machine import Controller, ControllerState, wait_for_group

from .camera import HebiCamera

if typing.TYPE_CHECKING:
    from collections.abc import Sequence

    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO
    from hebi.robot_model import RobotModel


class CameraMast(GroupManager):
    def __init__(self, pan_tilt_group: Group, hrdf: RobotModel | str):
        super().__init__(time.time, pan_tilt_group)

        if isinstance(hrdf, str):
            self.robot_model = hebi.robot_model.import_from_hrdf(hrdf)
        else:
            self.robot_model = hrdf

        self.output_frame = np.empty((4, 4), dtype=np.float64)

    @classmethod
    def from_config(cls, cfg: hebi.config.HebiConfig, lookup: hebi.Lookup) -> CameraMast:
        """Wait for the mast modules named in ``cfg`` and load its hrdf."""
        assert cfg.hrdf is not None, "mast config needs an hrdf"
        return cls(wait_for_group(lookup, cfg.families, cfg.names), cfg.hrdf)

    def _inner_update(self, t: float) -> bool:
        res = super()._inner_update(t)

        # Skip the frame update until the IMU reports an orientation; a NaN
        # quaternion would raise here and kill the control thread.
        quat = self.last_feedback[0].orientation
        if not np.any(np.isnan(quat)):
            base_frame = np.eye(4)
            base_frame[:3, :3] = R.from_quat([*quat[1:], quat[0]]).as_matrix()  # wxyz -> xyzw (scipy < 1.14)
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


class _ResumeToTeleop:
    """Mixin: after a disconnect or fault, resume in teleop instead of restarting this state's motion."""

    controller: MastController

    def resume_state(self):
        return CameraMastTeleopState(self.controller)


@dataclass(eq=False)
class CameraMastStartupState(ControllerState["MastController"]):
    # Never move on startup; home only when the user asks.
    def update(self, t_now, **kwargs):
        return CameraMastTeleopState(self.controller)


@dataclass(eq=False)
class CameraMastHomingState(_ResumeToTeleop, ControllerState["MastController"]):
    HOMING_TIME = 3.0  # s

    def on_enter(self, from_state, **kwargs):
        self.controller.mast.set_position(self.HOMING_TIME, self.controller.home_position)

    def update(self, t_now, **kwargs):
        if self.controller.mast.at_goal:
            return CameraMastTeleopState(self.controller)
        return None


@dataclass(eq=False)
class CameraMastTeleopState(_ResumeToTeleop, ControllerState["MastController"]):
    PAN_SCALING = 0.5
    TILT_SCALING = 0.5
    RAMP_TIME = 0.25  # s

    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state, **kwargs):
        self.controller.hold_position()

    def receive_inputs(self, t_now: float, inputs: MastInputs, **kwargs):
        self.controller.set_camera(inputs.zoom, inputs.light_enable, inputs.light_pwm)
        if self.requested is not None:
            return

        if inputs.home:
            self.requested = CameraMastHomingState(self.controller)
        else:
            v_pan = inputs.v_pan * self.PAN_SCALING
            v_tilt = inputs.v_tilt * self.TILT_SCALING
            self.controller.mast.set_velocity(self.RAMP_TIME, [v_pan, v_tilt])

    def update(self, t_now, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class CameraMastFaultState(ControllerState["MastController"]):
    # Drop the goal: a target held through the M-stop would be driven to on
    # release. Recovery resumes into Teleop, which holds from current feedback.
    def on_enter(self, from_state, **kwargs):
        self.controller.mast.cancel_goal()


@dataclass(eq=False)
class CameraMastExitState(ControllerState["MastController"]):
    def on_enter(self, from_state, **kwargs):
        self.controller.hold_position()


class MastController(Controller[CameraMast, MastInputs]):
    """Pan/tilt mast with a camera. Mobile IO disconnects hold position.

    ``camera`` is a ``HebiCamera`` or anything with the same methods.
    ``start()`` starts both the mast's and the camera's threads; ``stop()``
    stops both.
    """

    def __init__(
        self,
        mast: CameraMast,
        camera: HebiCamera,
        home_position: Sequence[float],
        starting_state: type[ControllerState] = CameraMastStartupState,
        fault_state: type[ControllerState] = CameraMastFaultState,
        exit_state: type[ControllerState] = CameraMastExitState,
        *,
        name: str = "Mast",
        input_timeout: float | None = 1.0,
    ):
        super().__init__(mast, starting_state, fault_state, exit_state, input_timeout=input_timeout, name=name)
        self.camera = camera
        self.home_position = np.asarray(home_position, dtype=np.float64)

    @property
    def mast(self) -> CameraMast:
        return self.manager

    def set_camera(self, zoom: float, light_enable: bool, light_pwm: float):
        # The camera's own thread sends pending_command; don't write it mid-send.
        with self.camera.lock:
            self.camera.zoom_level = zoom
            self.camera.light_enable = light_enable
            self.camera.light_pwm = light_pwm

    def start(self):
        if not self.camera.running:
            self.camera.start()
        try:
            super().start()
        except Exception:
            self.camera.stop()
            raise

    def stop(self):
        super().stop()
        if self.camera.running:
            self.camera.stop()


def create_mast_controller(cfg: hebi.config.HebiConfig, lookup: hebi.Lookup) -> MastController:
    """Find the mast and its camera. ``cfg.user_data`` gives ``camera_family`` and ``home_position``. Starts nothing."""
    mast = CameraMast.from_config(cfg, lookup)
    camera = HebiCamera.from_family(required_user_data(cfg, "camera_family"), lookup)
    return MastController(mast, camera, required_user_data(cfg, "home_position"))


def parse_mobile_feedback(m: MobileIO, timeout_ms: float = 100.0):
    if not m.update(timeout_ms):
        return None

    # Update Camera zoom/lights
    zoom = 1 + (m.get_axis_state(3) + 1.0) / 2.0
    light = (m.get_axis_state(4) + 1.0) / 2.0
    light_en = m.get_button_state(2)

    pan = -1.0 * m.get_axis_state(1)
    tilt = m.get_axis_state(2)

    return MastInputs(pan, tilt, m.get_button_state(1), zoom, light_en, light)


if __name__ == "__main__":
    config_dir = Path(__file__).absolute().parent / 'config'
    parser = config_arg_parser("Pan/tilt camera mast driven from a Mobile IO", config_dir)
    add_config_argument(parser, "config", "Mast config, e.g. pan_tilt_mast.cfg.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config, config_dir)
    family = cfg.families[0]
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving
    mast_control = create_mast_controller(cfg, lookup)
    camera = mast_control.camera
    camera.set_overlay_text('HEBI Mast Control Demo')

    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, family)
    while m is None:
        try:
            print('Waiting for Mobile IO device to come online...')
            time.sleep(1)
            m = create_mobile_io(lookup, family)
        except KeyboardInterrupt:
            sys.exit(0)

    m.update()
    m.send_layout(layout_file=str(config_dir / 'layouts' / 'CameraMastControl.json'))

    #######################
    ## Main Control Loop ##
    #######################

    mast_control.start()
    try:
        while mast_control.running:
            inputs = parse_mobile_feedback(m)
            if inputs:
                if m.get_button_state(8):
                    break
                mast_control.receive_inputs(inputs)
    except KeyboardInterrupt:
        pass
    finally:
        print('Quitting')
        mast_control.stop()
        camera.clear_overlay_text()
