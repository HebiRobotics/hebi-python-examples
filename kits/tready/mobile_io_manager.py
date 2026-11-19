from __future__ import annotations

import threading
from time import time
from typing import TYPE_CHECKING

import numpy as np

from kits.tready.treaded_base import (
    TreadyAligningState,
    TreadyCompliantFlippersState,
    TreadyDeployedState,
    TreadyDeployingState,
    TreadyFlatteningState,
    TreadyHomingState,
    TreadyRearingState,
    TreadyStartupState,
    TreadyStowingState,
    TreadyTeleopState,
)
from kits.tready.treaded_base_core import (
    ChassisVelocity,
    TreadyEStopState,
    TreadyExitState,
    TreadyInputs,
)
from kits.tready.tready_utils import set_mobile_io_instructions

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping
    from pathlib import Path
    from typing import Any, TypeAlias

    from hebi._internal.mobile_io import MobileIO

    from kits.common.state_machine import ControllerState

    from .treaded_base_core import TreadedBaseController

    # Parses one Mobile IO sample into one input per controller (None = no input this tick).
    MobileIOParser: TypeAlias = Callable[[MobileIO], tuple[Any, ...]]


class MobileIOUpdater:
    """Owns the Mobile IO for a demo: layouts, status text, and input parsing.

    Thread model:

    * The ``*_handler`` / ``update_*`` methods are registered on a controller
      (see ``attach``) and run on that controller's control thread. They only
      record what should change and never talk to the Mobile IO.
    * ``poll()`` runs on the main thread. It sends any pending layout, labels
      and text, then reads the Mobile IO and runs the parser for the active
      layout.

    :param demo_config: Maps a state class to ``(layout file, parser)``.
        Entering that state switches to the layout and parser. A layout of
        ``None`` keeps the current UI and only switches the parser.
    """

    CHARGE_NOW_VOLTAGE = 31.0  # [V]
    LOW_BATTERY_VOLTAGE = 32.0  # [V]

    def __init__(
        self,
        mobile_io: MobileIO,
        demo_config: Mapping[type[ControllerState], tuple[str | Path | None, MobileIOParser]],
    ):
        self.m = mobile_io
        self.layouts = {mode: vals[0] for mode, vals in demo_config.items()}
        self.parsers = {mode: vals[1] for mode, vals in demo_config.items()}
        self.active_parser: MobileIOParser | None = None

        # Shared between the control thread (handlers) and the main thread (poll).
        self._lock = threading.Lock()
        self._pending_mode: type[ControllerState] | None = None
        self._pending_axis_labels: dict[int, str] = {}
        self.base_msg = ""
        self.payload_msg = ""  # set by payload handlers (e.g. the sampler demo); guard with lock
        self.color = "transparent"
        self.voltage = 0.0
        self.voltage_update_time = 0.0

        self._last_msg = ""
        self._last_color = ""
        self._last_torque_labels: list[str] | None = None  # control thread only

    def attach(self, base_control: TreadedBaseController):
        """Register this updater's handlers on the base controller. Call before ``start()``."""
        base_control.add_transition_handler(self.base_transition_handler)
        base_control.add_update_handler(self.update_voltage_reading)
        base_control.add_update_handler(self.update_torque_mode)
        base_control.add_update_handler(self.update_startup_msg_base)

    # ------------------------------------------------------------------
    # Control thread: record changes only
    # ------------------------------------------------------------------

    def layout_transition_handler(self, controller, new_state: ControllerState):
        """Switch layout/parser if ``new_state`` has one. Register on any controller (e.g. a payload)."""
        with self._lock:
            for mode in self.layouts:
                if isinstance(new_state, mode):
                    self._pending_mode = mode
                    break

    def set_payload_status(self, msg: str, color: str | None = None):
        """Set the "Payload:" status line (and optionally the LED color). Safe from any thread."""
        with self._lock:
            self.payload_msg = msg
            if color is not None:
                self.color = color

    def base_transition_handler(self, controller: TreadedBaseController, new_state: ControllerState):
        self.layout_transition_handler(controller, new_state)
        with self._lock:
            match new_state:
                case TreadyDeployingState():
                    self.base_msg = "Robot Deploying\nPlease wait..."
                    self.color = "blue"

                case TreadyDeployedState() | TreadyStowingState():
                    self.base_msg = ""

                case TreadyHomingState():
                    self.base_msg = "Robot Homing Sequence\nPlease wait..."
                    self.color = "blue"

                case TreadyAligningState():
                    self.base_msg = "Robot Flippers Centering\nPlease wait..."
                    self.color = "blue"

                case TreadyFlatteningState():
                    self.base_msg = "Robot Flippers Flattening\nPlease wait..."
                    self.color = "blue"

                case TreadyRearingState():
                    self.base_msg = "Robot Rearing Up\nPlease wait..."
                    self.color = "blue"

                case TreadyStartupState() as startup:
                    self.base_msg = self._startup_msg(startup)
                    if startup.unlocked_flippers:
                        self.color = "yellow"

                case TreadyTeleopState():
                    self.base_msg = "Robot Ready to Control"
                    self.color = "green"

                case TreadyEStopState():
                    self.base_msg = "Emergency Stop Activated"
                    self.color = "red"

                case TreadyExitState():
                    self.base_msg = "Demo Stopped"
                    self.color = "red"

                case _:
                    self.base_msg = ""

    def update_torque_mode(self, controller: TreadedBaseController, state: ControllerState):
        """Queue the torque-mode slider labels while in compliant flipper mode.

        Compares against the labels last queued instead of using
        ``state.torque_labels_changed``: that flag is set per input on the main
        thread, so two inputs between control ticks would hide a change.
        """
        if not isinstance(state, TreadyCompliantFlippersState):
            self._last_torque_labels = None
            return
        labels = list(state.torque_labels)
        if labels != self._last_torque_labels:
            self._last_torque_labels = labels
            with self._lock:
                for i, label in enumerate(labels):
                    self._pending_axis_labels[i + 3] = label

    def update_startup_msg_base(self, controller: TreadedBaseController, state: ControllerState):
        if isinstance(state, TreadyStartupState):
            msg = self._startup_msg(state)
            with self._lock:
                self.base_msg = msg

    def update_voltage_reading(self, controller: TreadedBaseController, state: ControllerState):
        t_now = time()
        if t_now > self.voltage_update_time:
            self.voltage_update_time = t_now + 10.0  # update every 10 seconds
            voltage = float(np.mean(controller.base.last_feedback.voltage))
            with self._lock:
                self.voltage = voltage

    @staticmethod
    def _startup_msg(state: TreadyStartupState) -> str:
        return "     ".join(f"- Flipper {i + 1} is wound \n" for i in state.unlocked_flippers)

    # ------------------------------------------------------------------
    # Main thread: talk to the Mobile IO
    # ------------------------------------------------------------------

    def flush(self):
        """Send pending layout, labels and status text to the Mobile IO."""
        with self._lock:
            mode, self._pending_mode = self._pending_mode, None
            labels, self._pending_axis_labels = self._pending_axis_labels, {}
            # voltage is 0 until the first reading, which isn't a low battery
            if 0 < self.voltage < self.CHARGE_NOW_VOLTAGE:
                msg = f"CHARGE NOW! {self.voltage:.2f}V"
            elif 0 < self.voltage < self.LOW_BATTERY_VOLTAGE:
                msg = f"Low Battery: {self.voltage:.2f}V"
            else:
                msg = f"Battery: {self.voltage:.2f}V"
            if self.base_msg != "":
                msg += f"\nBase: {self.base_msg}"
            if self.payload_msg != "":
                msg += f"\nPayload: {self.payload_msg}"
            color = self.color

        if mode is not None:
            if (layout := self.layouts[mode]) is not None:
                print(f'Sending layout to Mobile IO: "{layout}"')
                self.m.send_layout(layout_file=str(layout))
                # A new layout clears the text display, so resend it.
                self._last_msg = ""
            self.active_parser = self.parsers[mode]

        for axis, label in labels.items():
            self.m.set_axis_label(axis, label, blocking=False)

        if msg != self._last_msg or color != self._last_color:
            set_mobile_io_instructions(self.m, msg, color)
            self._last_msg = msg
            self._last_color = color

    def poll(self, timeout_ms: float = 100.0) -> tuple[Any, ...] | None:
        """Flush pending UI changes, then parse one Mobile IO sample.

        Returns ``None`` if no new sample arrived within ``timeout_ms`` or no
        layout is active yet.
        """
        self.flush()
        if self.m.update(timeout_ms) and self.active_parser is not None:
            return self.active_parser(self.m)
        return None

    def parse_mobile_io_feedback(self, m: MobileIO | None = None, timeout_ms: float = 0.0):
        """Deprecated: use ``poll()``."""
        return self.poll(timeout_ms)


# ----------------------------------------------------------------------
# Parsers and shared parser pieces (main thread)
# ----------------------------------------------------------------------

TORQUE_MODE_SLIDER_VALUES = [0, -0.5, 1, 1]
FLIPPER_SLIDERS = (3, 4, 5, 6)
FLIPPER_LABELS = ("FL", "FR", "BL", "BR")


def handle_torque_toggle(m: MobileIO, torque_btn: int):
    """Switch the flipper sliders between velocity mode (snap to 0) and torque mode (free).

    Call on every sample, before any early return, so a toggle is never missed.
    """
    diff = m.get_button_diff(torque_btn)
    if diff == 1:
        for axis, value in zip(FLIPPER_SLIDERS, TORQUE_MODE_SLIDER_VALUES, strict=True):
            if not m.set_snap(axis, np.nan):
                print(f"Failed to set snap for axis {axis}")
            if not m.set_axis_value(axis, value):
                print(f"Failed to set axis value for axis {axis}")
    elif diff == -1:
        for axis, label in zip(FLIPPER_SLIDERS, FLIPPER_LABELS, strict=True):
            if not m.set_snap(axis, 0):
                print(f"Failed to set snap for axis {axis}")
            m.set_axis_label(axis, label, blocking=False)


def parse_flippers(m: MobileIO, height_up_btn: int, height_down_btn: int, sliders=FLIPPER_SLIDERS) -> list[float]:
    """Flipper velocity commands: the height buttons move all four, otherwise one slider per flipper.

    ``sliders`` has one axis per flipper (FL, FR, BL, BR); repeat an axis to
    drive two flippers from one slider.
    """
    height = m.get_button_state(height_up_btn) - m.get_button_state(height_down_btn)
    if height != 0:
        return [-height / 2] * 4
    return [m.get_axis_state(axis) for axis in sliders]


def torque_inputs(m: MobileIO, torque_btn: int) -> dict[str, bool]:
    """The ``torque_mode`` / ``torque_toggle`` fields of ``TreadyInputs``."""
    return {
        "torque_mode": bool(m.get_button_state(torque_btn)),
        "torque_toggle": m.get_button_diff(torque_btn) != 0,
    }


def update_startup_mode(m: MobileIO):
    # Startup buttons, joysticks, and sliders
    mapping = {
        "override_base_btn": 1,
        "unlock_flippers_btn": 3,
        "quit_demo_btn": 8,
    }

    quit = m.get_button_state(mapping["quit_demo_btn"])
    override_startup = m.get_button_state(mapping["override_base_btn"])

    return (TreadyInputs(quit=quit, override_startup=override_startup),)


def update_drive_mode(m: MobileIO):
    # Drive buttons, joysticks, and sliders
    mapping = {
        "reset_pose_btn": 1,
        "torque_btn": 2,
        "rear_up_btn": 3,
        "flatten_btn": 4,
        "height_up_btn": 5,
        "recenter_btn": 6,
        "height_down_btn": 7,
        "quit_demo_btn": 8,
        "turn_joy": 1,
        "forward_joy": 2,
    }

    handle_torque_toggle(m, mapping["torque_btn"])
    torque = torque_inputs(m, mapping["torque_btn"])

    if m.get_button_state(mapping["quit_demo_btn"]):
        return (TreadyInputs(quit=True),)
    if m.get_button_state(mapping["reset_pose_btn"]):
        return (TreadyInputs(home=True, **torque),)
    if m.get_button_state(mapping["recenter_btn"]):
        return (TreadyInputs(align_flippers=True, **torque),)
    if m.get_button_state(mapping["rear_up_btn"]):
        return (TreadyInputs(rear_up=True, **torque),)
    if m.get_button_state(mapping["flatten_btn"]):
        return (TreadyInputs(flatten_flippers=True, **torque),)

    return (
        TreadyInputs(
            base_motion=ChassisVelocity(
                m.get_axis_state(mapping["forward_joy"]),
                m.get_axis_state(mapping["turn_joy"]),
            ),
            flippers=parse_flippers(m, mapping["height_up_btn"], mapping["height_down_btn"]),
            **torque,
        ),
    )
