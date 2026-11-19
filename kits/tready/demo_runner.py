"""Shared setup and main loop for the Tready / Treadward demos.

A demo is: one Mobile IO, a ``MobileIOUpdater`` whose parsers turn each
Mobile IO sample into one input per controller, and a list of controllers
(the base first). Typical use::

    args = parse_args("My demo", arm_config="Arm config")
    cfg = load_config(args.base_config)
    arm_cfg = load_config(args.arm_config)
    lookup = hebi.Lookup()

    # 1. Find all hardware before anything starts moving.
    m = wait_for_mobile_io(lookup, cfg.families[0])
    base_control = create_base_controller(cfg, lookup)
    ...

    # 2. Wire up the UI, then 3. start and run.
    updater = MobileIOUpdater(m, {...})
    updater.attach(base_control)
    run(updater, [base_control, ...], log=not args.no_log)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

import hebi

from kits.common.config import add_config_argument, config_arg_parser
from kits.common.config import load_config as common_load_config

from .treaded_base import TreadyInitialState
from .treaded_base_core import TreadedBase, TreadedBaseController, TreadyEStopState, TreadyExitState

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from hebi._internal.lookup import Lookup
    from hebi.arm import Arm, Gripper

    from kits.common.state_machine import Controller

    from .mobile_io_manager import MobileIOUpdater

KITS_DIR = Path(__file__).absolute().parent.parent
CONFIG_DIR = KITS_DIR / "tready" / "config"
LAYOUT_DIR = CONFIG_DIR / "layouts"
LOG_DIR = KITS_DIR / "tready" / "logs"
# Config file names on the command line are looked up here, in order
CONFIG_DIRS = (CONFIG_DIR, KITS_DIR / "arms" / "config", KITS_DIR / "camera" / "config")


def demo_arg_parser(description: str, **systems: str) -> argparse.ArgumentParser:
    """Parser for a demo combining systems: ``BASE_CONFIG``, then one config per extra system, then ``--no-log``.

    ``systems`` maps each extra system's argument name to its help text, e.g.
    ``arm_config="Arm config, e.g. tready-arm-A-2240-06G.cfg.yaml"``.
    Add options (e.g. optional systems) to the returned parser before parsing.
    """
    parser = config_arg_parser(description, *CONFIG_DIRS)
    add_config_argument(parser, "base_config", "Treaded base config, e.g. tready.cfg.yaml or treadward.cfg.yaml")
    for name, help in systems.items():
        add_config_argument(parser, name, help)
    parser.add_argument("--no-log", action="store_true", help="Don't write HEBI logs to kits/tready/logs")
    return parser


def parse_args(description: str, argv: Sequence[str] | None = None, **systems: str) -> argparse.Namespace:
    """``demo_arg_parser(description, **systems).parse_args(argv)``."""
    return demo_arg_parser(description, **systems).parse_args(argv)


def load_config(name: str | Path) -> hebi.config.HebiConfig:
    """Load a config by path, or by file name from ``CONFIG_DIRS``."""
    return common_load_config(name, *CONFIG_DIRS)


def create_base_controller(cfg: hebi.config.HebiConfig, lookup: Lookup) -> TreadedBaseController:
    """Find the base modules and build its controller with the standard states. Does not start it."""
    base = TreadedBase.from_config(cfg, lookup)
    assert cfg.user_data is not None
    return TreadedBaseController(
        base,
        TreadyInitialState,
        TreadyEStopState,
        TreadyExitState,
        max_base_speed=cfg.user_data["max_base_speed"],
        name="Base",
    )


def create_arm(cfg: hebi.config.HebiConfig, lookup: Lookup) -> tuple[Arm, Gripper | None]:
    """Find the arm in ``cfg`` (raises if missing) and the gripper it declares, if any. Starts nothing."""
    from kits.arms.arm_controller import find_gripper

    arm = hebi.arm.create_from_config(cfg, lookup)
    return arm, find_gripper(lookup, cfg)


def run(
    updater: MobileIOUpdater,
    controllers: Sequence[Controller[Any, Any]],
    *,
    log: bool = True,
    log_dir: Path = LOG_DIR,
    poll_timeout_ms: float = 100.0,
    step: Callable[[float], bool] | None = None,
) -> None:
    """Start every controller, feed them Mobile IO inputs until quit, then stop them.

    ``controllers[0]`` must be the base controller. Each parser registered on
    ``updater`` returns a tuple with one entry per controller, in the same
    order. An entry of ``None`` means "no input for this controller this
    sample"; a controller that keeps getting ``None`` times out into its
    disconnected state. Parsers for the base's startup layout should
    therefore return explicit idle inputs for the other controllers, not
    ``None``.

    ``step(t_now)``, if given, runs once per loop iteration on the main
    thread, for things that aren't controllers here (e.g. an old-style
    ``update()``/``send()`` controller). Return ``False`` to end the demo.

    The demo ends when the base input has ``quit`` set, on Ctrl-C, when any
    controller stops running, or when ``step`` returns ``False``.
    """
    started: list[Controller[Any, Any]] = []
    try:
        for controller in controllers:
            if log:
                controller.start_logging(str(log_dir), controller.name.lower() or "log")
            controller.start()
            started.append(controller)

        while all(c.running for c in controllers):
            if step is not None and not step(time.time()):
                break
            inputs = updater.poll(poll_timeout_ms)
            if inputs is None:
                continue
            if len(inputs) != len(controllers):
                raise ValueError(f"Parser returned {len(inputs)} inputs for {len(controllers)} controllers")
            if getattr(inputs[0], "quit", False):
                break
            for controller, controller_input in zip(controllers, inputs, strict=True):
                if controller_input is not None:
                    controller.receive_inputs(controller_input)
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping demo")
        for controller in reversed(started):
            controller.stop()
        # Show the final state ("Demo Stopped") on the Mobile IO.
        updater.flush()
