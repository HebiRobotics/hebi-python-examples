from __future__ import annotations

from dataclasses import dataclass
import time
import datetime
import sys
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

import hebi

from .treaded_base_core import TreadyEStopState, TreadyExitState, TreadyInputs, TreadedBase, TreadedBaseController, ControllerState

from .mobile_io_manager import (
    MobileIOUpdater,
    update_startup_mode,
    update_drive_mode,
)

from .tready_utils import wait_for_mobile_io


@dataclass
class TreadyInitialState(ControllerState):
    def update(self, **kwargs):
        # On first loop move to startup
        self.controller.transition_to(TreadyStartupState(self.controller))


@dataclass
class TreadyStartupState(ControllerState):
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

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        # Transition to teleop when startup complete
        if tready_input.override_startup:
            self.allow_payload_startup = True

        if self.allow_payload_startup and tready_input.allow_startup:
            self.controller.transition_to(TreadyTeleopState(self.controller))


@dataclass
class TreadyHomingState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.home()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            self.controller.transition_to(TreadyTeleopState(self.controller))


@dataclass
class TreadyFlatteningState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.flatten_flippers()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            self.controller.transition_to(TreadyTeleopState(self.controller))


@dataclass
class TreadyRearingState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.rear_up()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            self.controller.transition_to(TreadyTeleopState(self.controller))


@dataclass
class TreadyDeployingState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.deploy()

    def update(self, **kwargs):
        if not self.controller.base.has_active_trajectory:
            self.controller.transition_to(TreadyDeployedState(self.controller))


@dataclass
class TreadyDeployedState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.payload_deploy_safe = True
        self.controller.payload_stow_safe = True

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        if tready_input.stow:
            self.controller.transition_to(TreadyStowingState(self.controller))

        if tready_input.drive_safe:
            # Mobile Base Control
            chassis_vels: NDArray[np.float64] = np.array(
                [
                    self.controller.max_base_speed * tready_input.base_motion.x,
                    0,
                    self.controller.max_chassis_rotation_vel * tready_input.base_motion.rz,
                ],
                dtype=np.float64,
            )

            duration = self.controller.base.chassis_ramp_time
        else:
            chassis_vels = np.zeros(3)
            duration = 0.2

        self.controller.base.set_chassis_vel_trajectory(duration, chassis_vels)

    def on_exit(self, to_state: ControllerState, **kwargs):
        self.controller.payload_deploy_safe = False
        self.controller.payload_stow_safe = False


@dataclass
class TreadyStowingState(ControllerState):
    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        if tready_input.stow_safe:
            self.controller.transition_to(TreadyTeleopState(self.controller))


@dataclass
class TreadyAligningState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.set_color("magenta")
        self.controller.base.align_flippers()

    def update(self, **kwargs):
        if not self.controller.base.has_active_flipper_trajectory:
            self.controller.transition_to(TreadyTeleopState(self.controller))


@dataclass
class TreadyTeleopState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.controller.base.clear_color()

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        # IDK why but sometimes this happens
        if not tready_input.allow_startup:
            print("How did this happen")
            self.controller.transition_to(TreadyStartupState(self.controller))

        # Flipper Control
        flipper_vels = -self.controller.base.flipper_sign * tready_input.flippers * self.controller.flipper_vel_scale
        self.controller.base.set_flipper_trajectory(self.controller.base.flipper_ramp_time, v=flipper_vels)

        if tready_input.torque_mode:
            self.controller.transition_to(TreadyCompliantFlippersState(self.controller))
        # Check for home button
        elif tready_input.home:
            self.controller.transition_to(TreadyHomingState(self.controller))
        # Check for flipper alignment
        elif tready_input.align_flippers:
            self.controller.transition_to(TreadyAligningState(self.controller))
        # Check for flipper flatten
        elif tready_input.flatten_flippers:
            self.controller.transition_to(TreadyFlatteningState(self.controller))
        # Check for rear up
        elif tready_input.rear_up:
            self.controller.transition_to(TreadyRearingState(self.controller))
        # Check for deployment
        elif tready_input.deploy:
            self.controller.transition_to(TreadyDeployingState(self.controller))

        if tready_input.drive_safe:
            # Mobile Base Control
            chassis_vels: NDArray[np.float64] = np.array(
                [
                    self.controller.max_base_speed * tready_input.base_motion.x,
                    0,
                    self.controller.max_chassis_rotation_vel * tready_input.base_motion.rz,
                ],
                dtype=np.float64,
            )

            duration = self.controller.base.chassis_ramp_time
        else:
            chassis_vels = np.zeros(3)
            duration = 0.2

        self.controller.base.set_chassis_vel_trajectory(duration, chassis_vels)


@dataclass
class TreadyCompliantFlippersState(ControllerState):
    def on_enter(self, from_state: ControllerState, **kwargs):
        self.torque_start_time = time.time()
        self.torque_labels = [
            "Max\nEff:",
            "Torque\nAngle:",
            "Roll:",
            "Pitch:",
        ]

    def receive_inputs(self, t_now: float, tready_input: TreadyInputs, **kwargs):
        if not tready_input.torque_mode:
            self.controller.transition_to(TreadyTeleopState(self.controller))
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

        if tready_input.drive_safe:
            # Mobile Base Control
            chassis_vels: NDArray[np.float64] = np.array(
                [
                    self.controller.max_base_speed * tready_input.base_motion.x,
                    0,
                    self.controller.max_chassis_rotation_vel * tready_input.base_motion.rz,
                ],
                dtype=np.float64,
            )

            duration = self.controller.base.chassis_ramp_time
        else:
            chassis_vels = np.zeros(3)
            duration = 0.2

        self.controller.base.set_chassis_vel_trajectory(duration, chassis_vels)


if __name__ == "__main__":
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
            TreadyStartupState: (layout_dir / "TreadwardStartupController.json", update_startup_mode),
            TreadyTeleopState: (layout_dir / "TreadwardDriveController.json", update_drive_mode),
        },
    )

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

    while base_control.running:
        try:
            inputs = updater.parse_mobile_io_feedback(m, timeout_ms=500)
            if inputs is not None:
                base_inputs = inputs[0]
                if base_inputs.quit:
                    break
                base_control.receive_inputs(base_inputs)

        except KeyboardInterrupt:
            break

    base_control.wait_for_shutdown(5)
