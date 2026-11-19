"""Treadward base with a core sampler payload, driven from one Mobile IO.

    python -m kits.tready.smart_treadward_sampler_control treadward.cfg.yaml treadward-core-sampler.cfg.yaml

Drive layout: "Deploy" stops the base and deploys the mast. Deployed layout:
the chain slider drives the sampler, "Wiggle" toggles the wiggler, and the
base may drive only while the chain is retracted. "Stow" retracts the chain,
stows the mast, then returns to driving.

Base and payload gate each other. ``couple()`` fills in each side's safety
inputs from the other controller's current state on every Mobile IO sample.
"""

from __future__ import annotations

import typing

import hebi

from kits.common.state_machine import wait_for_group

from .demo_runner import LAYOUT_DIR, create_base_controller, load_config, parse_args, run
from .mobile_io_manager import MobileIOUpdater, handle_torque_toggle, parse_flippers, torque_inputs
from .treaded_base import (
    TreadyDeployedState,
    TreadyDeployingState,
    TreadyInitialState,
    TreadyStartupState,
    TreadyStowingState,
    TreadyTeleopState,
)
from .treaded_base_core import ChassisVelocity, TreadedBaseController, TreadyInputs
from .treadward_sampler import (
    CoreSampler,
    CoreSamplerController,
    CoreSamplerInputs,
    SamplerDeployedState,
    SamplerDeployingState,
    SamplerFaultState,
    SamplerStartupState,
    SamplerStowedState,
    SamplerStowingState,
)
from .tready_utils import load_gains, wait_for_mobile_io

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


def couple(
    base_control: TreadedBaseController, payload_control: CoreSamplerController, base_in: TreadyInputs, payload_in: CoreSamplerInputs
):
    """Fill in the safety inputs each side needs from the other's current state."""
    base_state = base_control.state
    if isinstance(base_state, TreadyInitialState):
        base_allows_startup = False
    elif isinstance(base_state, TreadyStartupState):
        base_allows_startup = base_state.allow_payload_startup
    else:
        base_allows_startup = True

    payload_in.deploy_safe = isinstance(base_state, TreadyDeployedState)
    payload_in.stow_safe = isinstance(base_state, (TreadyDeployedState, TreadyStowingState))
    payload_in.allow_startup = base_allows_startup

    base_in.stow_safe = isinstance(payload_control.state, SamplerStowedState)
    base_in.allow_startup = payload_control.allow_base_startup
    base_in.drive_safe = payload_control.base_drive_safe
    return base_in, payload_in


def parse_startup(m: MobileIO):
    base_in = TreadyInputs(quit=bool(m.get_button_state(8)), override_startup=bool(m.get_button_state(1)))
    return base_in, CoreSamplerInputs(override_startup=bool(m.get_button_state(2)))


def parse_drive(m: MobileIO):
    buttons = {"reset": 1, "torque": 2, "deploy": 3, "flatten": 4, "height_up": 5, "recenter": 6, "height_down": 7, "quit": 8}
    handle_torque_toggle(m, buttons["torque"])
    torque = torque_inputs(m, buttons["torque"])

    if m.get_button_state(buttons["quit"]):
        return TreadyInputs(quit=True), CoreSamplerInputs()
    if m.get_button_state(buttons["reset"]):
        return TreadyInputs(home=True, **torque), CoreSamplerInputs()
    if m.get_button_state(buttons["recenter"]):
        return TreadyInputs(align_flippers=True, **torque), CoreSamplerInputs()
    if m.get_button_state(buttons["deploy"]):
        return TreadyInputs(deploy=True, **torque), CoreSamplerInputs(deploy=True)
    if m.get_button_state(buttons["flatten"]):
        return TreadyInputs(flatten_flippers=True, **torque), CoreSamplerInputs()

    base_in = TreadyInputs(
        base_motion=ChassisVelocity(m.get_axis_state(2), m.get_axis_state(1)),
        flippers=parse_flippers(m, buttons["height_up"], buttons["height_down"]),
        **torque,
    )
    return base_in, CoreSamplerInputs()


def parse_deploying(m: MobileIO):
    # Only quit is active while deploying/stowing.
    return TreadyInputs(quit=bool(m.get_button_state(8))), CoreSamplerInputs()


def parse_deployed(m: MobileIO):
    stow = bool(m.get_button_state(3))
    base_in = TreadyInputs(
        quit=bool(m.get_button_state(8)),
        stow=stow,
        base_motion=ChassisVelocity(m.get_axis_state(2), m.get_axis_state(1)),
    )
    payload_in = CoreSamplerInputs(
        stow=stow,
        chain=m.get_axis_state(6),
        wiggle_mode=bool(m.get_button_state(2)),
        wiggle_toggle=m.get_button_diff(2) != 0,
    )
    return base_in, payload_in


class SamplerStatus:
    """Payload status line on the Mobile IO. Handlers run on the payload's control thread."""

    def __init__(self, updater: MobileIOUpdater):
        self.updater = updater

    def on_transition(self, controller: CoreSamplerController, new_state):
        match new_state:
            case SamplerDeployingState():
                self.updater.set_payload_status("Payload Deploying\nPlease wait...")
            case SamplerDeployedState():
                self.updater.set_payload_status("Robot Ready to Control", "green")
            case SamplerStowingState():
                self.updater.set_payload_status("Payload Stowing\nPlease wait...", "blue")
            case SamplerStartupState():
                self.updater.set_payload_status(self.startup_msg(controller))
            case SamplerFaultState():
                self.updater.set_payload_status("Emergency Stop Activated", "red")
            case _:
                self.updater.set_payload_status("")

    def on_update(self, controller: CoreSamplerController, state):
        if isinstance(state, SamplerStartupState):
            self.updater.set_payload_status(self.startup_msg(controller))

    @staticmethod
    def startup_msg(controller: CoreSamplerController) -> str:
        problems = []
        if not controller.sampler.mast_stowed:
            problems.append("- Mast not stowed")
        if not controller.sampler.chain_stowed:
            problems.append("- Chain not stowed")
        return "\n".join(problems)


if __name__ == "__main__":
    args = parse_args("Treadward base with a core sampler payload", payload_config="Payload config, e.g. treadward-core-sampler.cfg.yaml")
    cfg = load_config(args.base_config)
    payload_cfg = load_config(args.payload_config)
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving
    m = wait_for_mobile_io(lookup, cfg.families[0])
    print("mobileIO device found.")
    base_control = create_base_controller(cfg, lookup)
    payload_group = wait_for_group(lookup, payload_cfg.families, payload_cfg.names)
    load_gains(payload_group, payload_cfg.gains["default"])
    payload_control = CoreSamplerController(CoreSampler(payload_group))

    def coupled(parser):
        def parse(m: MobileIO):
            return couple(base_control, payload_control, *parser(m))

        return parse

    updater = MobileIOUpdater(
        m,
        {
            TreadyStartupState: (LAYOUT_DIR / "TreadwardSamplerStartupController.json", coupled(parse_startup)),
            TreadyTeleopState: (LAYOUT_DIR / "TreadwardSamplerDriveController.json", coupled(parse_drive)),
            TreadyDeployingState: (LAYOUT_DIR / "TreadwardSamplerDeployingController.json", coupled(parse_deploying)),
            SamplerStowingState: (LAYOUT_DIR / "TreadwardSamplerDeployingController.json", coupled(parse_deploying)),
            SamplerDeployedState: (LAYOUT_DIR / "TreadwardSamplerDeployedController.json", coupled(parse_deployed)),
        },
    )
    updater.attach(base_control)
    status = SamplerStatus(updater)
    payload_control.add_transition_handler(updater.layout_transition_handler)
    payload_control.add_transition_handler(status.on_transition)
    payload_control.add_update_handler(status.on_update)

    run(updater, [base_control, payload_control], log=not args.no_log)
