# Update mobile io interface

from enum import Enum, auto

from kits.tready.treaded_base_core import (
    ChassisVelocity,
    TreadyControlState,
    TreadyInputs,
)
from kits.tready.tready import TreadyControl
from kits.tready.tready_utils import set_mobile_io_instructions

import numpy as np
from hebi._internal.mobile_io import MobileIO

from time import time


class MobileIOModes(Enum):
    STARTUP = auto()
    DRIVE = auto()


class MobileIOUpdater:
    def __init__(self, mobile_io: "MobileIO", startup_layout, drive_layout):

        self.m = mobile_io
        self.startup_layout = startup_layout
        self.drive_layout = drive_layout
        self.base_msg = ""
        self.color = ""
        self.last_msg = ""
        self.voltage = 0
        self.voltage_update_time = 0.0
        self.io_mode = MobileIOModes.STARTUP

    # Runs when base changes state
    def base_transition_handler(
        self, controller: TreadyControl, new_state: TreadyControlState
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
            self.m.send_layout(layout_file=self.startup_layout)
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
            self.m.send_layout(layout_file=self.drive_layout)
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
    def update_torque_mode(self, controller: TreadyControl, state: TreadyControlState):

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
        self, controller: TreadyControl, state: TreadyControlState
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
        self, controller: TreadyControl, state: TreadyControlState
    ):
        current_voltage = np.mean(controller.base.fbk.voltage)
        t_now = time()
        if t_now > self.voltage_update_time:
            self.voltage_update_time = t_now + 10.0  # update every 10 seconds
            self.voltage = current_voltage
            self.needs_update()

    def parse_mobile_io_feedback(self, m: "MobileIO"):

        io_mode = self.io_mode

        def update_startup_mode(m: "MobileIO"):
            # Startup buttons, joysticks, and sliders
            mapping = {
                "override_base_btn": 1,
                "unlock_flippers_btn": 3,
                "quit_demo_btn": 8,
            }

            if m.get_button_state(mapping["quit_demo_btn"]):
                return True, None, True

            return (
                False,
                TreadyInputs(
                    override_startup=m.get_button_state(mapping["override_base_btn"])
                ),
                True,
            )

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
                return True, None, True
            if m.get_button_state(mapping["reset_pose_btn"]):
                return (
                    False,
                    TreadyInputs(
                        home=True,
                        torque_mode=m.get_button_state(mapping["torque_btn"]),
                        torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
                    ),
                    True,
                )
            if m.get_button_diff(mapping["torque_btn"]) == 1:
                change_to_torque_mode(m)
            elif m.get_button_diff(mapping["torque_btn"]) == -1:
                change_to_velocity_mode(m)
            if m.get_button_state(mapping["recenter_btn"]):
                return (
                    False,
                    TreadyInputs(
                        align_flippers=True,
                        torque_mode=m.get_button_state(mapping["torque_btn"]),
                        torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
                    ),
                    True,
                )
            if m.get_button_state(mapping["rear_up_btn"]):
                b_inputs = TreadyInputs(
                    rear_up=True,
                    torque_mode=m.get_button_state(mapping["torque_btn"]),
                    torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
                )
                return False, b_inputs, True
            if m.get_button_state(mapping["flatten_btn"]):
                return (
                    False,
                    TreadyInputs(
                        flatten_flippers=True,
                        torque_mode=m.get_button_state(mapping["torque_btn"]),
                        torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
                    ),
                    True,
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
                False,
                TreadyInputs(
                    base_motion=chassis_velocity,
                    flippers=flippers,
                    torque_mode=m.get_button_state(mapping["torque_btn"]),
                    torque_toggle=(m.get_button_diff(mapping["torque_btn"]) != 0.0),
                ),
                True,
            )

        if m.update(0.0):
            if io_mode == MobileIOModes.DRIVE:
                return update_drive_mode(m)
            elif io_mode == MobileIOModes.STARTUP:
                return update_startup_mode(m)

        return False, None, False
