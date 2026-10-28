from __future__ import annotations

from enum import Enum, auto
from time import time

from kits.tready.treaded_base import (
    TreadyAligningState,
    TreadyCompliantFlippersState,
    TreadyEStopState,
    TreadyExitState,
    TreadyFlatteningState,
    TreadyHomingState,
    TreadyRearingState,
    TreadyStartupState,
    TreadyTeleopState,
)

from .treaded_base_core import (
    ChassisVelocity,
    ControllerState,
    TreadyInputs,
)
from kits.tready.tready_utils import set_mobile_io_instructions

import numpy as np

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Callable, Mapping
    from pathlib import Path
    from hebi._internal.mobile_io import MobileIO
    from .treaded_base import TreadedBaseController


class MobileIOModes(Enum):
    STARTUP = auto()
    DRIVE = auto()
    DEPLOYING = auto()
    DEPLOYED = auto()
    ARM = auto()


class MobileIOUpdater:
    def __init__(
        self,
        mobile_io: MobileIO,
        demo_config: Mapping[type[ControllerState], tuple[str | Path, Callable[[MobileIO], tuple]]],
    ):

        self.m = mobile_io
        self.layouts = {mode: vals[0] for mode, vals in demo_config.items()}
        self.parsers = {mode: vals[1] for mode, vals in demo_config.items()}
        self.base_msg = ""
        self.color = "transparent"
        self.last_msg = ""
        self.voltage = 0
        self.voltage_update_time = 0.0
        self.active_parser = None

    # Runs when base changes state
    def base_transition_handler(self, controller: TreadedBaseController, new_state: ControllerState):
        if isinstance(new_state, tuple(self.layouts.keys())):
            state_cls = type(new_state)
            print(f'Sending layout to Mobile IO: "{self.layouts[state_cls]}"')
            self.m.send_layout(layout_file=str(self.layouts[state_cls]))
            self.active_parser = self.parsers[state_cls]

        match new_state:
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
                if len(startup.unlocked_flippers) > 0:
                    self.color = "yellow"
                    self.base_msg = "     ".join(f"- Flipper {i + 1} is wound \n" for i in startup.unlocked_flippers)
                else:
                    self.base_msg = ""

            case TreadyTeleopState():
                self.base_msg = "Robot Ready to Control"
                self.color = "green"

            case TreadyEStopState():
                self.base_msg = "Emergency Stop Activated"
                self.color = "red"

            case TreadyExitState():
                self.base_msg = "Demo Stopped"
                self.payload_msg = ""
                self.color = "red"

            case _:
                self.base_msg = ""

        self.needs_update()

    # Called to update the message on the mobileIO display
    def needs_update(self):
        msg = f"Battery: {self.voltage:.2f}V"
        if self.base_msg != "":
            msg += f"\nBase: {self.base_msg}"

        if msg != self.last_msg:
            set_mobile_io_instructions(self.m, msg, self.color)

        self.last_msg = msg

    # Updates axis labels for torque mode sliders
    def update_torque_mode(self, controller: TreadedBaseController, state: ControllerState):

        if isinstance(state, TreadyCompliantFlippersState):
            if state.torque_labels_changed:
                for i, label in enumerate(state.torque_labels):
                    self.m.set_axis_label(i + 3, label, blocking=False)

    # Updates the base startup message
    def update_startup_msg_base(self, controller: TreadedBaseController, state: ControllerState):
        if isinstance(state, TreadyStartupState):
            if len(state.unlocked_flippers) > 0:
                self.base_msg = "     ".join(f"- Flipper {i + 1} is wound \n" for i in state.unlocked_flippers)
            else:
                self.base_msg = ""
            self.needs_update()

    # Updates the voltage reading
    def update_voltage_reading(self, controller: TreadedBaseController, state: ControllerState):
        t_now = time()
        if t_now > self.voltage_update_time:
            self.voltage_update_time = t_now + 10.0  # update every 10 seconds
            self.voltage = np.mean(controller.base.last_feedback.voltage)
            self.needs_update()

    def parse_mobile_io_feedback(self, m: MobileIO, timeout_ms=0.0):

        if m.update(timeout_ms):
            if self.active_parser is not None:
                return self.active_parser(m)

        return None


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
        "front_left_slider": 3,
        "front_right_slider": 4,
        "back_left_slider": 5,
        "back_right_slider": 6,
    }

    def change_to_torque_mode(m: MobileIO):
        axis_vals = [0, -0.5, 1, 1]
        for i in range(3, 7):
            if not m.set_snap(i, np.nan):
                print(f"Failed to set snap for axis {i}")
            if not m.set_axis_value(i, axis_vals[i - 3]):
                print(f"Failed to set axis value for axis {i}")

    def change_to_velocity_mode(m: MobileIO):
        for i in range(3, 7):
            if not m.set_snap(i, 0):
                print(f"Failed to set snap for axis {i}")
        m.set_axis_label(mapping["front_left_slider"], "FL", blocking=False)
        m.set_axis_label(mapping["front_right_slider"], "FR", blocking=False)
        m.set_axis_label(mapping["back_left_slider"], "BL", blocking=False)
        m.set_axis_label(mapping["back_right_slider"], "BR", blocking=False)

    if m.get_button_state(mapping["quit_demo_btn"]):
        return (TreadyInputs(quit=True),)
    if m.get_button_state(mapping["reset_pose_btn"]):
        return (
            TreadyInputs(
                home=True,
                torque_mode=m.get_button_state(mapping["torque_btn"]),
                torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
            ),
        )
    if m.get_button_diff(mapping["torque_btn"]) == 1:
        change_to_torque_mode(m)
    elif m.get_button_diff(mapping["torque_btn"]) == -1:
        change_to_velocity_mode(m)
    if m.get_button_state(mapping["recenter_btn"]):
        return (
            TreadyInputs(
                align_flippers=True,
                torque_mode=m.get_button_state(mapping["torque_btn"]),
                torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
            ),
        )
    if m.get_button_state(mapping["rear_up_btn"]):
        return (
            TreadyInputs(
                rear_up=True,
                torque_mode=m.get_button_state(mapping["torque_btn"]),
                torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
            ),
        )
    if m.get_button_state(mapping["flatten_btn"]):
        return (
            TreadyInputs(
                flatten_flippers=True,
                torque_mode=m.get_button_state(mapping["torque_btn"]),
                torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
            ),
        )

    chassis_velocity = ChassisVelocity(
        m.get_axis_state(mapping["forward_joy"]),
        m.get_axis_state(mapping["turn_joy"]),
    )
    height_up = m.get_button_state(mapping["height_up_btn"])
    height_down = m.get_button_state(mapping["height_down_btn"])
    height = height_up - height_down
    if height != 0:
        flippers = [-height / 2] * 4
    else:
        flippers = [
            m.get_axis_state(mapping["front_left_slider"]),
            m.get_axis_state(mapping["front_right_slider"]),
            m.get_axis_state(mapping["back_left_slider"]),
            m.get_axis_state(mapping["back_right_slider"]),
        ]

    return (
        TreadyInputs(
            base_motion=chassis_velocity,
            flippers=flippers,
            torque_mode=m.get_button_state(mapping["torque_btn"]),
            torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
        ),
    )
