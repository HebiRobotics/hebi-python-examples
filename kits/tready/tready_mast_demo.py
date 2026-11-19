#! /usr/bin/env python3
"""Drive a Tready / Treadward base with a pan/tilt camera mast from one Mobile IO.

    python -m kits.tready.tready_mast_demo tready.cfg.yaml pan_tilt_mast.cfg.yaml --stream-to 10.10.2.1

``--stream-to`` sends the camera's video to that host (the address the old
demo hard-coded is shown above).
"""

from __future__ import annotations

import typing

import hebi

from ..camera.pan_tilt_mast import MastInputs, create_mast_controller
from .demo_runner import LAYOUT_DIR, create_base_controller, demo_arg_parser, load_config, run
from .mobile_io_manager import MobileIOUpdater, update_drive_mode, update_startup_mode
from .treaded_base import TreadyStartupState, TreadyTeleopState
from .tready_utils import wait_for_mobile_io

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO

# Mast inputs while the base is starting up: hold still, lights off.
MAST_IDLE = MastInputs(v_pan=0.0, v_tilt=0.0, home=False, zoom=1.0, light_enable=False, light_pwm=0.2)


def update_camera_drive_mode(m: MobileIO):
    (base_inputs,) = update_drive_mode(m)

    pan = -1.0 * m.get_axis_state(7)
    tilt = m.get_axis_state(8)
    mast_inputs = MastInputs(pan, tilt, base_inputs.home, 1.0, True, 0.4)
    return base_inputs, mast_inputs


def update_camera_startup_mode(m: MobileIO):
    (base_inputs,) = update_startup_mode(m)
    return base_inputs, MAST_IDLE


if __name__ == "__main__":
    parser = demo_arg_parser("Drive a treaded base with a pan/tilt camera mast", mast_config="Mast config, e.g. pan_tilt_mast.cfg.yaml")
    parser.add_argument("--stream-to", metavar="HOST", help="Stream the camera's video to this host")
    parser.add_argument("--stream-kbps", type=int, default=1000, help="Video bitrate when streaming (default: 1000)")
    args = parser.parse_args()
    cfg = load_config(args.base_config)
    mast_cfg = load_config(args.mast_config)
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving
    m = wait_for_mobile_io(lookup, cfg.families[0])
    print("mobileIO device found.")
    base_control = create_base_controller(cfg, lookup)
    mast_control = create_mast_controller(mast_cfg, lookup)
    camera = mast_control.camera
    if args.stream_to:
        camera.stream_to(args.stream_to, args.stream_kbps)
    camera.set_overlay_text('HEBI Mast Control Demo')

    updater = MobileIOUpdater(
        m,
        {
            TreadyStartupState: (LAYOUT_DIR / "TreadwardStartupController.json", update_camera_startup_mode),
            TreadyTeleopState: (LAYOUT_DIR / "TreadwardCameraDriveController.json", update_camera_drive_mode),
        },
    )
    updater.attach(base_control)

    try:
        run(updater, [base_control, mast_control], log=not args.no_log)
    finally:
        camera.clear_overlay_text()
