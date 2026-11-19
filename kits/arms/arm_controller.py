"""Shared pieces for arm controllers built on ``kits.common.state_machine``."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from time import sleep
from typing import TYPE_CHECKING, Any, Generic, TypeVar

import hebi
import numpy as np

from kits.common.state_machine import Controller, ControllerState, DisconnectedState

if TYPE_CHECKING:
    from collections.abc import Sequence

    from hebi.arm import Arm, Gripper
    from numpy.typing import NDArray


CONFIG_DIR = Path(__file__).absolute().parent / "config"


I = TypeVar("I")


class ArmController(Controller["Arm", I], Generic[I]):
    """Base for arm controllers.

    Owns an optional gripper. The gripper is *not* attached with
    ``arm.set_end_effector``: without ``aux`` values in every goal, the arm
    reports each update as failed. Instead:

    * In normal states the gripper command is sent every tick (an update
      handler: not while disconnected, faulted or exiting), with the group's
      usual command lifetime.
    * On disconnect, the gripper group's command lifetime is set to 0
      (persist until the next command) and the command is sent once. Nothing
      more is sent while disconnected, and a held object isn't released when
      the old lifetime would have expired.
    * The first normal tick after recovering restores the lifetime.
    """

    def __init__(
        self,
        arm: Arm,
        starting_state: type[ControllerState[Any]],
        fault_state: type[ControllerState[Any]],
        exit_state: type[ControllerState[Any]],
        *,
        gripper: Gripper | None = None,
        name: str = "Arm",
        input_timeout: float | None = 1.0,
    ):
        super().__init__(arm, starting_state, fault_state, exit_state, input_timeout=input_timeout, name=name)
        self.gripper = gripper
        self._saved_gripper_lifetime_ms: int | None = None
        self.add_transition_handler(self._set_led_color)
        if gripper is not None:
            self.add_update_handler(self._send_gripper)

    @property
    def arm(self) -> Arm:
        return self.manager

    @staticmethod
    def _set_led_color(controller: ArmController[Any], new_state: ControllerState[Any]) -> None:
        """The same arm colors in every demo: blue when disconnected, the state's ``led_color`` otherwise."""
        if isinstance(new_state, DisconnectedState):
            color = "blue"
        else:
            color = getattr(new_state, "led_color", "transparent")
        controller.arm.pending_command.led.color = color

    @property
    def _gripper_group(self):
        # hebi.arm.Gripper has no public accessor for its group.
        return self.gripper._group

    def on_disconnect(self) -> None:
        super().on_disconnect()
        if self.gripper is not None and self._saved_gripper_lifetime_ms is None:
            group = self._gripper_group
            self._saved_gripper_lifetime_ms = group.command_lifetime
            group.command_lifetime = 0
            self.gripper.send()

    def _send_gripper(self, controller, state) -> None:
        # Update handlers don't run while disconnected, faulted or exiting.
        if self._saved_gripper_lifetime_ms is not None:
            self._gripper_group.command_lifetime = self._saved_gripper_lifetime_ms
            self._saved_gripper_lifetime_ms = None
        self.gripper.send()

    def set_gripper(self, closed: bool) -> None:
        if self.gripper is None:
            return
        is_closed = self.gripper.state == 1.0
        if closed and not is_closed:
            print(f"{self.prefix}Gripper close")
            self.gripper.close()
        elif not closed and is_closed:
            print(f"{self.prefix}Gripper open")
            self.gripper.open()

    def move_to(self, position: Sequence[float] | NDArray[np.float64], duration: float) -> None:
        """Joint-space move to ``position`` over ``duration`` seconds."""
        goal = hebi.arm.Goal(self.arm.size)
        goal.add_waypoint(t=duration, position=position)
        self.arm.set_goal(goal)

    def current_position(self) -> NDArray[np.float64]:
        """Commanded position if the modules report one, else measured position (a copy)."""
        pos = self.arm.last_feedback.position_command
        if np.any(np.isnan(pos)):
            pos = self.arm.last_feedback.position
        return np.array(pos, dtype=np.float64)


@dataclass(eq=False)
class ArmFaultState(ControllerState[ArmController[Any]]):
    # M-stopped modules ignore commands. Drop the goal rather than hold one:
    # a held target from before the M-stop would be driven to on release.
    # Recovery resumes into a state that holds from current feedback.
    def on_enter(self, from_state, **kwargs):
        self.controller.arm.cancel_goal()


@dataclass(eq=False)
class ArmExitState(ControllerState[ArmController[Any]]):
    def on_enter(self, from_state, **kwargs):
        self.controller.hold_position()


def user_data_value(cfg: hebi.config.HebiConfig, *keys: str, default: Any = None) -> Any:
    """First of ``keys`` present in ``cfg.user_data`` (configs name the same setting differently)."""
    user_data = cfg.user_data or {}
    for key in keys:
        if key in user_data:
            return user_data[key]
    return default


def find_gripper(lookup: hebi.Lookup, cfg: hebi.config.HebiConfig, attempts: int = 3) -> Gripper | None:
    """Find the gripper an arm config declares, or ``None`` if it declares none or it isn't found.

    Reads the convention of the ``*G`` arm configs: ``has_gripper: true`` in
    ``user_data`` and gripper gains as ``gains: gripper:``. Also accepts
    ``gripper_name`` / ``gripper_family`` / ``gripper_gains`` in ``user_data``
    (as in ``ex_AR_kit_w_gripper.cfg.yaml``). Efforts come from
    ``gripper_close_effort`` / ``gripper_open_effort``. Not finding a declared
    gripper after ``attempts`` tries is a warning, so a demo can run without it.
    """
    name = user_data_value(cfg, "gripper_name")
    if name is None:
        if not user_data_value(cfg, "has_gripper", default=False):
            return None
        name = "gripperSpool"
    family = user_data_value(cfg, "gripper_family", default=cfg.families[0])

    group = None
    for _ in range(attempts):
        if (group := lookup.get_group_from_names([family], [name])) is not None:
            break
        print(f"Looking for gripper module {family}/{name}...")
        sleep(1)
    if group is None:
        print(f"WARNING: gripper {family}/{name} not found, continuing without it")
        return None

    gripper = hebi.arm.Gripper(
        group,
        float(user_data_value(cfg, "gripper_close_effort", default=-5.0)),
        float(user_data_value(cfg, "gripper_open_effort", default=1.0)),
    )
    gains = user_data_value(cfg, "gripper_gains", default=(cfg.gains or {}).get("gripper"))
    if gains is not None:
        gripper.load_gains(os.path.join(cfg.config_location, gains))
    gripper.open()
    return gripper
