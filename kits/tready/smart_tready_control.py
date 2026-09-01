import hebi
from hebi.util import create_mobile_io
from time import time, sleep
import datetime
import os
from os.path import join

from .mobile_io_manager import MobileIOUpdater, MobileIOModes, update_startup_mode, update_drive_mode
from .treaded_base_core import TreadyInputs
from .tready_utils import load_gains

from .tready import TreadyControl, TreadyBase


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
    lookup = hebi.Lookup()
    sleep(2)

    # Treaded base family & names
    family = "Tready"
    flipper_names = [f"T{n + 1}_J1_flipper" for n in range(4)]
    wheel_names = [f"T{n + 1}_J2_track" for n in range(4)]

    # mobileIO setup
    print("Looking for mobileIO device...")
    m = create_mobile_io(lookup, family)
    while m is None:
        print("Waiting for mobileIO device to come online...")
        sleep(1)
        m = create_mobile_io(lookup, family)

    print("mobileIO device found.")
    m.update()

    # Create base group
    base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)
    while base_group is None:
        print("Looking for Tready modules...")
        sleep(1)
        base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)

    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config')
    load_gains(base_group, os.path.join(config_dir, "gains", "smart-tready-gains.xml"))

    base = TreadyBase(base_group, chassis_ramp_time=0.5, flipper_ramp_time=0.1)
    base.set_robot_model(os.path.join(config_dir, "hrdf", "Tready.hrdf"))
    base_control = TreadyControl(base)

    layout_dir = join(config_dir, "layouts")
    startup_layout = join(layout_dir, "TreadwardStartupController.json")
    drive_layout = join(layout_dir, "TreadwardDriveController.json")

    mio_demo_config = {
        MobileIOModes.STARTUP: (startup_layout, update_startup_mode),
        MobileIOModes.DRIVE: (drive_layout, update_drive_mode),
    }

    updater = MobileIOUpdater(m, mio_demo_config)
    base_control._transition_handlers.append(updater.base_transition_handler)

    base_control._update_handlers.append(updater.update_voltage_reading)
    base_control._update_handlers.append(updater.update_torque_mode)
    base_control._update_handlers.append(updater.update_startup_msg_base)

    # can enable start logging here
    logging = True

    if logging:
        tready_dir = os.path.dirname(__file__)
        now = datetime.datetime.now()
        base.group.start_log(
            os.path.join(tready_dir, "logs"), f"base_{now:%Y-%m-%d-%H:%M:%S}"
        )

    while base_control.running:
        t = time()
        try:
            inputs = updater.parse_mobile_io_feedback(m)
            if inputs is None:
                m_update = False
                base_inputs = update_inputs()
            else:
                m_update = True
                base_inputs = update_inputs(inputs[0])

            if base_inputs.quit:
                break

            base_control.update(t, m_update, base_inputs)
            base_control.send()

        except KeyboardInterrupt:
            break

    base_control.stop()

    if logging:
        base.group.stop_log()
