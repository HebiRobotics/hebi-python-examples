"""Tready-Sub: a Tready base with optional leader/follower arms and an optional pan/tilt mast.

    python -m kits.tready.tready_sub_control tready_sub.cfg.yaml \
        --arms tready_sub_leader_arm.cfg.yaml tready_sub_follower_arm.cfg.yaml \
        --mast tready_sub_mast.cfg.yaml

Uses the default Mobile IO layout, labeled in ``setup_mobile_io``. The arms
and the camera mast are optional: leave out ``--arms`` / ``--mast`` to run
without them. Systems that are given must be present.

Buttons: 1 reset (homes base and mast; overrides base startup while starting
up), 2 torque mode, 3 follow, 4 haptic feedback, 5/7 flipper height,
6 recenter flippers, 8 quit. Left stick drives, right stick pans/tilts.
"""

from __future__ import annotations

import typing

import hebi

from kits.arms.ex_leader_follower import LeaderFollowerController, LeaderFollowerInputs
from kits.camera.pan_tilt_mast import MastInputs, create_mast_controller
from kits.common.config import add_config_argument, required_user_data

from .demo_runner import create_base_controller, demo_arg_parser, load_config, run
from .mobile_io_manager import MobileIOUpdater, handle_torque_toggle, parse_flippers, torque_inputs
from .treaded_base import TreadyStartupState, TreadyTeleopState
from .treaded_base_core import ChassisVelocity, TreadyInputs
from .tready_utils import wait_for_mobile_io

if typing.TYPE_CHECKING:
    from collections.abc import Callable

    from hebi._internal.mobile_io import MobileIO

BUTTONS = {"reset": 1, "torque": 2, "follow": 3, "haptic": 4, "height_up": 5, "recenter": 6, "height_down": 7, "quit": 8}
AXES = {"turn": 1, "forward": 2, "pan": 7, "tilt": 8}

MAST_IDLE = MastInputs(v_pan=0.0, v_tilt=0.0, home=False, zoom=1.0, light_enable=False, light_pwm=0.0)
ARM_IDLE = LeaderFollowerInputs()


def setup_mobile_io(m: MobileIO):
    labels = {
        BUTTONS["reset"]: "⟲",
        BUTTONS["torque"]: "Torque",
        BUTTONS["follow"]: "Follow",
        BUTTONS["haptic"]: "Haptic",
        BUTTONS["height_up"]: "⤾◼⤿",
        BUTTONS["recenter"]: "Center",
        BUTTONS["height_down"]: "⤿◼⤾",
        BUTTONS["quit"]: "❌",
    }
    for button, label in labels.items():
        m.set_button_label(button, label)
    for button in (BUTTONS["torque"], BUTTONS["follow"], BUTTONS["haptic"]):
        m.set_button_mode(button, 1)
    m.set_button_output(BUTTONS["follow"], 1)
    m.set_button_output(BUTTONS["haptic"], 1)

    for axis, label in {1: "Turn", 2: "Drive", 3: "FL", 4: "FR", 5: "BL", 6: "BR", 7: "Pan", 8: "Tilt"}.items():
        m.set_axis_label(axis, label)
    for axis in range(1, 9):
        m.set_snap(axis, 0)


def parse_drive(m: MobileIO) -> TreadyInputs:
    handle_torque_toggle(m, BUTTONS["torque"])
    torque = torque_inputs(m, BUTTONS["torque"])

    if m.get_button_state(BUTTONS["quit"]):
        return TreadyInputs(quit=True)
    if m.get_button_state(BUTTONS["reset"]):
        return TreadyInputs(home=True, **torque)
    if m.get_button_state(BUTTONS["recenter"]):
        return TreadyInputs(align_flippers=True, **torque)
    return TreadyInputs(
        base_motion=ChassisVelocity(m.get_axis_state(AXES["forward"]), m.get_axis_state(AXES["turn"])),
        flippers=parse_flippers(m, BUTTONS["height_up"], BUTTONS["height_down"]),
        **torque,
    )


def make_parsers(has_arms: bool, has_mast: bool) -> tuple[Callable, Callable]:
    """Startup and drive parsers returning one input per controller present (base, [arms], [mast])."""

    def others(arm_in: LeaderFollowerInputs, mast_in: MastInputs) -> tuple:
        return (*([arm_in] if has_arms else []), *([mast_in] if has_mast else []))

    def parse_startup(m: MobileIO):
        base_in = TreadyInputs(quit=bool(m.get_button_state(BUTTONS["quit"])), override_startup=bool(m.get_button_state(BUTTONS["reset"])))
        return (base_in, *others(ARM_IDLE, MAST_IDLE))

    def parse_drive_mode(m: MobileIO):
        arm_in = LeaderFollowerInputs(
            follow=m.get_button_state(BUTTONS["follow"]) == 1,
            haptic_fbk=m.get_button_state(BUTTONS["haptic"]) == 1,
        )
        mast_in = MastInputs(
            v_pan=-m.get_axis_state(AXES["pan"]),
            v_tilt=-m.get_axis_state(AXES["tilt"]),
            home=bool(m.get_button_state(BUTTONS["reset"])),
            zoom=1.0,
            light_enable=False,
            light_pwm=0.0,
        )
        return (parse_drive(m), *others(arm_in, mast_in))

    return parse_startup, parse_drive_mode


if __name__ == "__main__":
    parser = demo_arg_parser("Tready-Sub: base with optional leader/follower arms and camera mast")
    add_config_argument(
        parser,
        "arms",
        "Leader and follower arm configs, e.g. tready_sub_leader_arm.cfg.yaml tready_sub_follower_arm.cfg.yaml",
        option=True,
        nargs=2,
    )
    add_config_argument(parser, "mast", "Camera mast config, e.g. tready_sub_mast.cfg.yaml", option=True)
    args = parser.parse_args()
    cfg = load_config(args.base_config)
    leader_cfg, follower_cfg = (load_config(name) for name in args.arms) if args.arms else (None, None)
    mast_cfg = load_config(args.mast) if args.mast else None
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving. Systems named on the command line are required.
    m = wait_for_mobile_io(lookup, cfg.families[0])
    print("mobileIO device found.")
    m.update()
    setup_mobile_io(m)

    base_control = create_base_controller(cfg, lookup)
    arm_control = None
    if leader_cfg is not None:
        leader = hebi.arm.create_from_config(leader_cfg, lookup)
        follower = hebi.arm.create_from_config(follower_cfg, lookup)
        arm_control = LeaderFollowerController(follower, leader, home_position=required_user_data(follower_cfg, "home_position"))
    mast_control = create_mast_controller(mast_cfg, lookup) if mast_cfg is not None else None

    parse_startup, parse_drive_mode = make_parsers(arm_control is not None, mast_control is not None)
    updater = MobileIOUpdater(
        m,
        {
            TreadyStartupState: (None, parse_startup),
            TreadyTeleopState: (None, parse_drive_mode),
        },
    )
    updater.attach(base_control)

    controllers = [c for c in (base_control, arm_control, mast_control) if c is not None]
    run(updater, controllers, log=not args.no_log)
