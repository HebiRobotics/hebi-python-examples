"""Tready base plus a MAPS input arm driving a follower arm with a gripper.

    python -m kits.tready.tready_leader_follower_control tready.cfg.yaml maps-input-arm.cfg.yaml tready-maps-follower-arm.cfg.yaml

The base's Mobile IO drives the base. A second Mobile IO, in the follower
arm config's ``mobile_io_family``, controls the gripper and MAPS reset. The MAPS
leader/follower and gripper controllers come from
``advanced/demos/MAPS_control`` and still use the old ``update()``/``send()``
style, so they run in the main loop via ``run(step=...)``.
"""

from __future__ import annotations

import os
from time import sleep

import hebi
import numpy as np
from hebi.util import create_mobile_io

from advanced.demos.MAPS_control.gripper_control import GripperControl
from advanced.demos.MAPS_control.MAPS_input_device_example import (
    ContinuousAngleMaps,
    LeaderFollowerControl,
    LeaderFollowerControlState,
)
from advanced.demos.MAPS_control.MAPS_input_device_w_gripper_example import parse_mobile_feedback, setup_mobile_io
from kits.arms.arm_controller import find_gripper
from kits.common.config import required_user_data, user_data

from .demo_runner import LAYOUT_DIR, create_base_controller, load_config, parse_args, run
from .mobile_io_manager import MobileIOUpdater, update_drive_mode, update_startup_mode
from .treaded_base import TreadyStartupState, TreadyTeleopState
from .tready_utils import wait_for_mobile_io

# Allowed angular difference (deg) per joint before starting to align
ALLOWED_DIFF = np.array([30.0, 20.0, 30.0, 20.0, 45.0, 45.0, 360.0])


def show(m, msg: str):
    m.clear_text()
    m.add_text(msg)


def wait_for_group_with_message(lookup: hebi.Lookup, m, families, names, msg: str):
    while (group := lookup.get_group_from_names(families, names)) is None:
        show(m, msg)
        sleep(1)
    return group


def find_output_arm(cfg: hebi.config.HebiConfig, lookup: hebi.Lookup, m):
    """Follower arm from its config, with the double-shoulder mirror module synced to joint 1."""
    arm = hebi.arm.create_from_config(cfg, lookup)  # also loads the default gains

    # Keep the two modules in the double shoulder bracket in sync:
    # mirror joint index 1 onto the config's mirror_module.
    mirror_group = wait_for_group_with_message(
        lookup, m, cfg.families, [user_data(cfg, "mirror_module")], "Still looking for mirror group..."
    )
    arm.add_plugin(hebi.arm.DoubledJointMirror(1, mirror_group))
    gains_cmd = hebi.GroupCommand(1)
    gains_cmd.read_gains(os.path.join(cfg.config_location, user_data(cfg, "mirror_gains")))
    mirror_group.send_command_with_acknowledgement(gains_cmd)

    arm.cancel_goal()
    return arm


if __name__ == "__main__":
    args = parse_args(
        "Tready base with a MAPS-driven follower arm and gripper",
        maps_config="MAPS input device config, e.g. maps-input-arm.cfg.yaml",
        follower_arm_config="Follower arm config, e.g. tready-maps-follower-arm.cfg.yaml",
    )
    cfg = load_config(args.base_config)
    maps_cfg = load_config(args.maps_config)
    follower_cfg = load_config(args.follower_arm_config)
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving
    m_base = wait_for_mobile_io(lookup, cfg.families[0])
    print("Base mobileIO device found.")
    base_control = create_base_controller(cfg, lookup)

    arm_mio_family = required_user_data(follower_cfg, "mobile_io_family")
    print("Looking for arm Mobile IO...")
    while (m_arm := create_mobile_io(lookup, arm_mio_family)) is None:
        print("Waiting for arm Mobile IO device to come online...")
        sleep(1)
    setup_mobile_io(m_arm)

    maps_group = wait_for_group_with_message(
        lookup,
        m_arm,
        maps_cfg.families,
        maps_cfg.names,
        "MAPS arm not found: Check connection and make sure all modules are blinking green",
    )
    input_arm = ContinuousAngleMaps(maps_group, np.array(user_data(maps_cfg, "angle_offsets"), dtype=np.float64))
    output_arm = find_output_arm(follower_cfg, lookup, m_arm)

    if (gripper := find_gripper(lookup, follower_cfg)) is None:
        raise RuntimeError("This demo needs the follower arm's gripper")

    input_arm.update()
    output_arm.update()

    leader_follower_control = LeaderFollowerControl(input_arm, output_arm, required_user_data(follower_cfg, "home_position"), ALLOWED_DIFF)
    gripper_control = GripperControl(gripper)

    # Arm UI (old-style controller: handlers run on the main thread)
    def update_arm_ui(controller: LeaderFollowerControl, new_state: LeaderFollowerControlState):
        if controller.state == new_state:
            return
        if new_state == LeaderFollowerControlState.HOMING:
            m_arm.set_led_color("blue", blocking=False)
            show(m_arm, "Homing...")
        elif new_state == LeaderFollowerControlState.UNALIGNED:
            m_arm.set_led_color("blue", blocking=False)
        elif new_state == LeaderFollowerControlState.ALIGNED:
            m_arm.set_led_color("green", blocking=False)
            show(m_arm, "Aligned!")

    leader_follower_control._transition_handlers.append(update_arm_ui)

    last_text_update = 0.0

    def step_arm(t_now: float) -> bool:
        """One iteration of the MAPS leader/follower + gripper loop."""
        global last_text_update
        arm_inputs, gripper_inputs = parse_mobile_feedback(m_arm)
        leader_follower_control.update(t_now, arm_inputs)
        gripper_control.update(t_now, gripper_inputs)
        leader_follower_control.send()
        gripper_control.send()

        if leader_follower_control.state == LeaderFollowerControlState.UNALIGNED and t_now - last_text_update > 0.1:
            last_text_update = t_now
            show(m_arm, f"Unaligned: {np.around(np.rad2deg(leader_follower_control.angle_diff), decimals=0)}")
        return leader_follower_control.running and gripper_control.running

    updater = MobileIOUpdater(
        m_base,
        {
            TreadyStartupState: (LAYOUT_DIR / "TreadwardStartupController.json", update_startup_mode),
            TreadyTeleopState: (LAYOUT_DIR / "TreadwardDriveController.json", update_drive_mode),
        },
    )
    updater.attach(base_control)

    # The MAPS loop blocks on arm feedback each step, so don't also block on the Mobile IO.
    run(updater, [base_control], log=not args.no_log, step=step_arm, poll_timeout_ms=0)
