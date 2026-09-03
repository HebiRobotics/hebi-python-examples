# Update mobile io interface

from enum import Enum, auto
from typing import Callable
from time import time

from kits.tready.treaded_base_core import (
    ChassisVelocity,
    TreadyControlState,
    TreadyInputs,
    TreadedBaseControl,
)
from kits.tready.tready_utils import set_mobile_io_instructions

import numpy as np
from hebi._internal.mobile_io import MobileIO


class MobileIOModes(Enum):
    STARTUP = auto()
    DRIVE = auto()
    DEPLOYING = auto()
    DEPLOYED = auto()
    ARM = auto()


class MobileIOUpdater:
    def __init__(
        self,
        mobile_io: "MobileIO",
        demo_config: dict[MobileIOModes, tuple[str, Callable[[MobileIO], tuple]]],
    ):

        self.m = mobile_io
        self.layouts = {mode: vals[0] for mode, vals in demo_config.items()}
        self.parsers = {mode: vals[1] for mode, vals in demo_config.items()}
        self.base_msg = ""
        self.color = ""
        self.last_msg = ""
        self.voltage = 0
        self.voltage_update_time = 0.0
        self.io_mode = MobileIOModes.STARTUP

    @property
    def io_mode(self):
        return self._io_mode

    @io_mode.setter
    def io_mode(self, value: MobileIOModes):
        self._io_mode = value
        print(f'Sending layout to Mobile IO: "{self.layouts[value]}"')
        self.m.send_layout(layout_file=self.layouts[value])

    # Runs when base changes state
    def base_transition_handler(
        self, controller: TreadedBaseControl, new_state: TreadyControlState
    ):
        if controller.state == new_state:
            return

        if new_state is TreadyControlState.HOMING:
            controller.base.set_color("magenta")
            self.base_msg = "Robot Homing Sequence\nPlease wait..."
            self.color = "blue"

        elif new_state is TreadyControlState.ALIGNING:
            controller.base.set_color("magenta")
            self.base_msg = "Robot Flippers Centering\nPlease wait..."
            self.color = "blue"

        elif new_state is TreadyControlState.FLATTENING:
            controller.base.set_color("magenta")
            self.base_msg = "Robot Flippers Flattening\nPlease wait..."
            self.color = "blue"

        elif new_state is TreadyControlState.REARING:
            controller.base.set_color("magenta")
            self.base_msg = "Robot Rearing Up\nPlease wait..."
            self.color = "blue"

        elif new_state is TreadyControlState.STARTUP:
            controller.base.set_color("magenta")
            self.io_mode = MobileIOModes.STARTUP
            self.base_msg = ""
            for i in range(4):
                if controller.base.flipper_wound[i]:
                    if self.base_msg == "":
                        self.base_msg += "- Flipper " + str(i + 1) + " is wound \n"
                    else:
                        self.base_msg += (
                            "      - Flipper " + str(i + 1) + " is wound \n"
                        )

            self.color = "yellow"

        elif new_state is TreadyControlState.TELEOP:
            controller.base.clear_color()
            self.io_mode = MobileIOModes.DRIVE
            self.m.set_axis_label(
                6, "BR"
            )  # TODO: this should not be needed but send layout has a bug
            self.base_msg = "Robot Ready to Control"
            self.color = "green"

        elif new_state is TreadyControlState.DISCONNECTED:
            print("Lost connection to Controller. Please reconnect.")
            controller.base.set_color("blue")

        elif new_state is TreadyControlState.EMERGENCY_STOP:
            controller.base.set_color("yellow")
            self.base_msg = "Emergency Stop Activated"
            self.color = "red"

        elif new_state is TreadyControlState.EXIT:
            controller.base.set_color("red")
            self.base_msg = "Demo Stopped"
            self.payload_msg = ""
            self.color = "red"

        else:
            self.base_msg = ""

        self.needs_update()

    # Called to update the message on the mobileIO display
    def needs_update(self):
        if self.base_msg != "":
            msg = f"Battery: {self.voltage:.2f}V\nBase: {self.base_msg}"
        else:
            msg = f"Battery: {self.voltage:.2f}V"

        if msg != self.last_msg:
            set_mobile_io_instructions(self.m, msg, self.color)

        self.last_msg = msg

    # Updates axis labels for torque mode sliders
    def update_torque_mode(
        self, controller: TreadedBaseControl, state: TreadyControlState
    ):

        if controller.state is TreadyControlState.TELEOP:
            """
            if self.m.get_button_diff(2) == 1:
                self.m.set_axis_label(3, 'Torque', blocking=False)
                self.m.set_axis_label(4, 'Angle', blocking=False)
                self.m.set_axis_label(5, 'Pitch', blocking=False)
                self.m.set_axis_label(6, 'Roll', blocking=False)
            elif self.m.get_button_diff(2) == -1:
                self.m.set_axis_label(3, 'FL', blocking=False)
                self.m.set_axis_label(4, 'FR', blocking=False)
                self.m.set_axis_label(5, 'BL', blocking=False)
                self.m.set_axis_label(6, 'BR', blocking=False)
            """
            if controller.torque_labels is not None:
                if controller.torque_labels_changed:
                    for i, label in enumerate(controller.torque_labels):
                        self.m.set_axis_label(i + 3, label, blocking=False)
            else:
                self.m.set_axis_label(3, "FL", blocking=False)
                self.m.set_axis_label(4, "FR", blocking=False)
                self.m.set_axis_label(5, "BL", blocking=False)
                self.m.set_axis_label(6, "BR", blocking=False)

    # Updates the base startup message
    def update_startup_msg_base(
        self, controller: TreadedBaseControl, state: TreadyControlState
    ):
        if state is TreadyControlState.STARTUP:
            self.base_msg = ""
            for i in range(4):
                if controller.base.flipper_wound[i]:
                    if self.base_msg == "":
                        self.base_msg += "- Flipper " + str(i + 1) + " is wound \n"
                    else:
                        self.base_msg += (
                            "          - Flipper " + str(i + 1) + " is wound \n"
                        )

            self.needs_update()

    # Updates the voltage reading
    def update_voltage_reading(
        self, controller: TreadedBaseControl, state: TreadyControlState
    ):
        t_now = time()
        if t_now > self.voltage_update_time:
            self.voltage_update_time = t_now + 10.0  # update every 10 seconds
            self.voltage = np.mean(controller.base.fbk.voltage)
            self.needs_update()

    def parse_mobile_io_feedback(self, m: "MobileIO"):

        if m.update(0.0):
            if self.io_mode in self.parsers:
                return self.parsers[self.io_mode](m)

        return None


def update_startup_mode(m: "MobileIO"):
    # Startup buttons, joysticks, and sliders
    mapping = {
        "override_base_btn": 1,
        "unlock_flippers_btn": 3,
        "quit_demo_btn": 8,
    }

    quit = m.get_button_state(mapping["quit_demo_btn"])
    override_startup = m.get_button_state(mapping["override_base_btn"])

    return (TreadyInputs(quit=quit, override_startup=override_startup),)


def update_drive_mode(m: "MobileIO"):
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

    def change_to_torque_mode(m: "MobileIO"):
        axis_vals = [0, -0.5, 1, 1]
        for i in range(3, 7):
            if not m.set_snap(i, np.nan):
                print(f"Failed to set snap for axis {i}")
            if not m.set_axis_value(i, axis_vals[i - 3]):
                print(f"Failed to set axis value for axis {i}")

    def change_to_velocity_mode(m: "MobileIO"):
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
