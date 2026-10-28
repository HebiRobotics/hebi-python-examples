#! /usr/bin/env python3
from __future__ import annotations

import datetime
import sys
from pathlib import Path

import hebi

from .tready_utils import wait_for_mobile_io

from ..camera.camera import HebiCamera
from ..camera.pan_tilt_mast import (
    CameraMast,
    MastController,
    CameraMastStartupState,
    CameraMastDisconnectedState,
    CameraMastExitState,
    MastInputs,
)
from .treaded_base_core import (
    TreadedBaseController,
    TreadyEStopState,
    TreadyExitState,
)
from .treaded_base import TreadedBase, TreadyInitialState, TreadyStartupState, TreadyTeleopState

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


if __name__ == "__main__":
    from .mobile_io_manager import MobileIOUpdater, update_startup_mode, update_drive_mode

    def update_camera_drive_mode(m: MobileIO):
        base_inputs = update_drive_mode(m)[0]

        pan = -1.0 * m.get_axis_state(7)
        tilt = m.get_axis_state(8)
        mast_inputs = MastInputs(pan, tilt, base_inputs.home, 1.0, True, 0.4)
        return base_inputs, mast_inputs

    def update_camera_startup_mode(m: MobileIO):
        return update_startup_mode(m)[0], MastInputs(0.0, 0.0, False, 1.0, False, 0.2)

    lookup = hebi.Lookup()

    config_dir = Path(__file__).absolute().parent / "config"
    cfg = hebi.config.load_config(str(config_dir / sys.argv[1]))

    m = wait_for_mobile_io(lookup, cfg.families[0])

    print("mobileIO device found.")
    m.update()

    layout_dir = config_dir / "layouts"
    updater = MobileIOUpdater(
        m,
        {
            TreadyStartupState: (layout_dir / "TreadwardStartupController.json", update_camera_startup_mode),
            TreadyTeleopState: (layout_dir / "TreadwardCameraDriveController.json", update_camera_drive_mode),
        },
    )

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

    mast_control = MastController(mast, camera, CameraMastStartupState, CameraMastDisconnectedState, CameraMastExitState)

    base = TreadedBase.from_config(cfg)
    base_control = TreadedBaseController(
        base, TreadyInitialState, TreadyEStopState, TreadyExitState, max_base_speed=cfg.user_data["max_base_speed"]
    )

    base_control._transition_handlers.append(updater.base_transition_handler)

    base_control._update_handlers.append(updater.update_voltage_reading)
    base_control._update_handlers.append(updater.update_torque_mode)
    base_control._update_handlers.append(updater.update_startup_msg_base)

    # can enable start logging here
    logging = True

    if logging:
        log_dir = Path(__file__).parent / "logs"
        base.group.start_log(str(log_dir), f"base_{datetime.datetime.now():%Y-%m-%d-%H:%M:%S}")

    #######################
    ## Main Control Loop ##
    #######################

    while base_control.running and mast_control.running:
        try:
            inputs = updater.parse_mobile_io_feedback(m, timeout_ms=500)
            if inputs is not None:
                base_inputs, mast_inputs = inputs

                if base_inputs.quit:
                    break

                base_control.receive_inputs(base_inputs)
                mast_control.receive_inputs(mast_inputs)
        except KeyboardInterrupt:
            break

    print('Quitting')
    camera.clear_overlay_text()
    mast_control.wait_for_shutdown(2)
    base_control.wait_for_shutdown(5)
