from time import time, sleep
import datetime
from os.path import join, dirname, abspath

import hebi

from .treaded_base_core import (
    TreadedBase,
    TreadedBaseConfig,
    TreadedBaseControl,
    TreadyInputs,
)
from .mobile_io_manager import (
    MobileIOUpdater,
    MobileIOModes,
    update_startup_mode,
    update_drive_mode,
)

from .tready_utils import wait_for_mobile_io, try_create_base_group


def update_inputs(base_inputs: "TreadyInputs | None" = None):
    if base_inputs is None:
        base_inputs = TreadyInputs()

    base_inputs.deploy_safe = True
    base_inputs.stow_safe = True
    base_inputs.payload_deployed = False
    base_inputs.allow_startup = True
    base_inputs.drive_safe = True

    return base_inputs


if __name__ == "__main__":
    config_dir = join(dirname(abspath(__file__)), "config")

    tready_config = TreadedBaseConfig(
        hrdf_file=join(config_dir, "hrdf", "Tready.hrdf"),
        gains_file=join(config_dir, "gains", "smart-tready-gains.xml"),
        wheel_diameter=0.125,  # m
        wheel_base=0.285,  # m
        torso_torque_scale=2.5,  # Nm
        torque_mode_max=25,  # Nm
    )

    layout_dir = join(config_dir, "layouts")
    startup_layout = join(layout_dir, "TreadwardStartupController.json")
    drive_layout = join(layout_dir, "TreadwardDriveController.json")

    mio_demo_config = {
        MobileIOModes.STARTUP: (startup_layout, update_startup_mode),
        MobileIOModes.DRIVE: (drive_layout, update_drive_mode),
    }

    lookup = hebi.Lookup()
    sleep(2)

    family = "Tready"

    m = wait_for_mobile_io(lookup, family)

    print("mobileIO device found.")
    m.update()

    # Create base group
    base_group = try_create_base_group(lookup, family)
    while base_group is None:
        print(f"Looking for {family} modules...")
        sleep(1)
        base_group = try_create_base_group(lookup, family)

    base = TreadedBase(
        tready_config, base_group, chassis_ramp_time=0.5, flipper_ramp_time=0.1
    )
    base_control = TreadedBaseControl(base, max_base_speed=0.25)

    updater = MobileIOUpdater(m, mio_demo_config)
    base_control._transition_handlers.append(updater.base_transition_handler)

    base_control._update_handlers.append(updater.update_voltage_reading)
    base_control._update_handlers.append(updater.update_torque_mode)
    base_control._update_handlers.append(updater.update_startup_msg_base)

    # can enable start logging here
    logging = True

    if logging:
        tready_log_dir = join(dirname(__file__), "logs")
        now = datetime.datetime.now()
        base.group.start_log(tready_log_dir, f"base_{now:%Y-%m-%d-%H:%M:%S}")

    while base_control.running:
        t = time()
        try:
            inputs = updater.parse_mobile_io_feedback(m)
            if inputs is None:
                base_inputs = None
            else:
                base_inputs = update_inputs(inputs[0])
                if base_inputs.quit:
                    break

            base_control.update(t, base_inputs)
            base_control.send()

        except KeyboardInterrupt:
            break

    base_control.stop()

    if logging:
        base.group.stop_log()
