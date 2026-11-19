from __future__ import annotations

from time import time
import datetime
from pathlib import Path

import hebi

from .treaded_base_core import (
    TreadedBase,
    TreadedBaseControl,
    TreadyInputs,
)
from .mobile_io_manager import (
    MobileIOUpdater,
    MobileIOModes,
    update_startup_mode,
    update_drive_mode,
)

from .tready_utils import wait_for_mobile_io


def update_inputs(base_inputs: TreadyInputs | None = None):
    if base_inputs is None:
        base_inputs = TreadyInputs()

    base_inputs.deploy_safe = True
    base_inputs.stow_safe = True
    base_inputs.payload_deployed = False
    base_inputs.allow_startup = True
    base_inputs.drive_safe = True

    return base_inputs


if __name__ == "__main__":
    lookup = hebi.Lookup()

    config_dir = Path(__file__).absolute().parent / "config"
    cfg = hebi.config.load_config(str(config_dir / 'tready.cfg.yaml'))

    m = wait_for_mobile_io(lookup, cfg.families[0])

    print("mobileIO device found.")
    m.update()

    layout_dir = config_dir / "layouts"
    updater = MobileIOUpdater(
        m,
        {
            MobileIOModes.STARTUP: (layout_dir / "TreadwardStartupController.json", update_startup_mode),
            MobileIOModes.DRIVE: (layout_dir / "TreadwardDriveController.json", update_drive_mode),
        },
    )

    base = TreadedBase.from_config(cfg)
    base_control = TreadedBaseControl(base, max_base_speed=cfg.user_data["max_base_speed"])

    base_control._transition_handlers.append(updater.base_transition_handler)

    base_control._update_handlers.append(updater.update_voltage_reading)
    base_control._update_handlers.append(updater.update_torque_mode)
    base_control._update_handlers.append(updater.update_startup_msg_base)

    # can enable start logging here
    logging = True

    if logging:
        tready_log_dir = Path(__file__).parent / "logs"
        base.group.start_log(str(tready_log_dir), f"base_{datetime.datetime.now():%Y-%m-%d-%H:%M:%S}")

    while base_control.running:
        try:
            inputs = updater.parse_mobile_io_feedback(m, timeout_ms=500)
            if inputs is not None:
                base_inputs = update_inputs(inputs[0])
                if base_inputs.quit:
                    break
                base_control.update(time(), base_inputs)

        except KeyboardInterrupt:
            break

    base_control.stop()

    if logging:
        base.group.stop_log()
