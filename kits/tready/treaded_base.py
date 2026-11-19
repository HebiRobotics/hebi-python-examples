from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np

from kits.common.state_machine import ControllerState

from .treaded_base_core import TreadyInputs, TreadedBaseController

# Re-exported so demos can import the base and its standard states from one place.
from .treaded_base_core import TreadedBase, TreadyEStopState, TreadyExitState  # noqa: F401


class _ResumeToTeleop:
    """Mixin: after a disconnect or fault, resume in teleop instead of restarting this state's motion."""

    controller: TreadedBaseController

    def resume_state(self):
        return TreadyTeleopState(self.controller)


@dataclass(eq=False)
class TreadyInitialState(ControllerState):
    def update(self, **kwargs):
        # On first loop move to startup
        return TreadyStartupState(self.controller)


@dataclass(eq=False)
class TreadyStartupState(ControllerState):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def resume_state(self):
        return TreadyStartupState(self.controller)

    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.unlocked_flippers: list[int] = []
        if any(self.controller.base.flipper_wound):
            self.allow_payload_startup = False
            for i in range(4):
                if self.controller.base.flipper_wound[i]:
                    self.unlocked_flippers.append(i)
        else:
            self.allow_payload_startup = True

    def update(self, **kwargs):
        if len(self.unlocked_flippers) == 0:
            self.allow_payload_startup = True

        if not self.allow_payload_startup:
            self.controller.base.unlock_flippers(self.unlocked_flippers)

        requested, self.requested = self.requested, None
        return requested

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        # Transition to teleop when startup complete
        if tready_input.override_startup:
            self.allow_payload_startup = True

        if self.allow_payload_startup and tready_input.allow_startup:
            self.requested = TreadyTeleopState(self.controller)


@dataclass(eq=False)
class TreadyHomingState(_ResumeToTeleop, ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.home()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            return TreadyTeleopState(self.controller)
        return None


@dataclass(eq=False)
class TreadyFlatteningState(_ResumeToTeleop, ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.flatten_flippers()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            return TreadyTeleopState(self.controller)
        return None


@dataclass(eq=False)
class TreadyRearingState(_ResumeToTeleop, ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.rear_up()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            return TreadyTeleopState(self.controller)
        return None


@dataclass(eq=False)
class TreadyDeployingState(ControllerState):
    """Stop the chassis so a payload can deploy. Payload coupling lives in the demo (it reads this state)."""

    def resume_state(self):
        return TreadyDeployingState(self.controller)

    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.deploy()

    def update(self, **kwargs):
        if not self.controller.base.has_active_trajectory:
            return TreadyDeployedState(self.controller)
        return None


@dataclass(eq=False)
class TreadyDeployedState(ControllerState):
    """Payload deployed. Driving is allowed only while the payload says it's safe (``drive_safe``)."""

    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def resume_state(self):
        return TreadyDeployedState(self.controller)

    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.clear_color()

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        if self.requested is not None:
            return
        if tready_input.stow:
            self.requested = TreadyStowingState(self.controller)
            return

        self.controller.drive(tready_input)

    def update(self, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class TreadyStowingState(ControllerState):
    """Hold still until the payload reports it is stowed (``stow_safe``)."""

    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def resume_state(self):
        return TreadyStowingState(self.controller)

    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.hold_motion()

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        if tready_input.stow_safe:
            self.requested = TreadyTeleopState(self.controller)

    def update(self, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class TreadyAligningState(_ResumeToTeleop, ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.align_flippers()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            return TreadyTeleopState(self.controller)
        return None


@dataclass(eq=False)
class TreadyTeleopState(_ResumeToTeleop, ControllerState):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.clear_color()

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        if self.requested is not None:
            return

        # IDK why but sometimes this happens
        if not tready_input.allow_startup:
            print("How did this happen")
            self.requested = TreadyStartupState(self.controller)
            return

        # Flipper Control
        flipper_vels = -self.controller.base.flipper_sign * tready_input.flippers * self.controller.flipper_vel_scale
        self.controller.base.set_flipper_trajectory(self.controller.base.flipper_ramp_time, v=flipper_vels)

        if tready_input.torque_mode:
            self.requested = TreadyCompliantFlippersState(self.controller)
        # Check for home button
        elif tready_input.home:
            self.requested = TreadyHomingState(self.controller)
        # Check for flipper alignment
        elif tready_input.align_flippers:
            self.requested = TreadyAligningState(self.controller)
        # Check for flipper flatten
        elif tready_input.flatten_flippers:
            self.requested = TreadyFlatteningState(self.controller)
        # Check for rear up
        elif tready_input.rear_up:
            self.requested = TreadyRearingState(self.controller)
        # Check for deployment
        elif tready_input.deploy:
            self.requested = TreadyDeployingState(self.controller)
        else:
            self.controller.drive(tready_input)

    def update(self, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class TreadyCompliantFlippersState(_ResumeToTeleop, ControllerState):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state: ControllerState, **kwargs):
        self.torque_start_time = time.time()
        self.torque_labels = [
            "Max\nEff:",
            "Torque\nAngle:",
            "Roll:",
            "Pitch:",
        ]
        self.torque_labels_changed = True

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        if self.requested is not None:
            return
        if not tready_input.torque_mode:
            self.requested = TreadyTeleopState(self.controller)
            return

        torque_slider = (tready_input.flippers[0] + 1) / 2
        angle_slider = (tready_input.flippers[1] + 1) / 2
        roll_slider = (tready_input.flippers[2] + 1) / 2
        pitch_slider = (tready_input.flippers[3] + 1) / 2

        config = self.controller.base.config

        torque_max = config.torque_mode_max * torque_slider

        target_flipper_angle = angle_slider * 2 * config.torque_angle_offset
        roll_angle, pitch_angle, _ = self.controller.base.pose

        roll_err = roll_angle
        pitch_err = pitch_angle

        roll_p_gain = config.torso_torque_scale * roll_slider
        pitch_p_gain = config.torso_torque_scale * pitch_slider

        signs = np.empty((4, 2))
        signs[:, 0] = [0, -1, 0, 1] if roll_err > 0 else [-1, 0, 1, 0]
        signs[:, 1] = [1, -1, 0, 0] if pitch_err > 0 else [0, 0, 1, -1]

        leveling_torque = (signs @ np.array([roll_p_gain * roll_err, pitch_p_gain * pitch_err])).flatten()

        flipper_angle_error = self.controller.base.flipper_sign * target_flipper_angle - self.controller.base.flipper_fbk.position
        flipper_efforts = torque_max * np.tanh(flipper_angle_error) + leveling_torque
        flipper_efforts *= min(
            (t_now - self.torque_start_time) / config.torque_ramp_time,
            1,
        )

        self.controller.base.set_flipper_torque(flipper_efforts)

        self.prev_torque_labels = self.torque_labels
        self.torque_labels = [
            f"Max\nEff:\n{np.round(torque_max, 2)}",
            f"Torque\nAngle:\n{np.round(target_flipper_angle, 2)}",
            f"Roll:\n{np.round(roll_slider)}",
            f"Pitch:\n{np.round(pitch_slider)}",
        ]
        self.torque_labels_changed = self.prev_torque_labels != self.torque_labels

        self.controller.drive(tready_input)

    def update(self, **kwargs):
        requested, self.requested = self.requested, None
        return requested


if __name__ == "__main__":
    # Run as `python -m kits.tready.treaded_base BASE_CONFIG`. This file is executing as __main__, which is a
    # second copy of the state classes above, and the Mobile IO updater matches on the package's copies,
    # so everything the demo uses is imported from the package.
    import hebi

    from kits.tready.demo_runner import LAYOUT_DIR, create_base_controller, load_config, parse_args, run
    from kits.tready.mobile_io_manager import MobileIOUpdater, update_drive_mode, update_startup_mode
    from kits.tready.treaded_base import TreadyStartupState, TreadyTeleopState
    from kits.tready.tready_utils import wait_for_mobile_io

    args = parse_args("Drive a Tready / Treadward base with a Mobile IO")
    cfg = load_config(args.base_config)
    lookup = hebi.Lookup()

    # Find all hardware before anything starts moving
    m = wait_for_mobile_io(lookup, cfg.families[0])
    print("mobileIO device found.")
    base_control = create_base_controller(cfg, lookup)

    updater = MobileIOUpdater(
        m,
        {
            TreadyStartupState: (LAYOUT_DIR / "TreadwardStartupController.json", update_startup_mode),
            TreadyTeleopState: (LAYOUT_DIR / "TreadwardDriveController.json", update_drive_mode),
        },
    )
    updater.attach(base_control)

    run(updater, [base_control], log=not args.no_log)
