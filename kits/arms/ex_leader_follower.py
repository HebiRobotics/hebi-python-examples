"""Leader/follower: a compliant leader arm drives a follower arm, with optional haptic feedback.

States: Startup -> Idle -> Aligning -> Follow, and Homing on the home button.
Both arms hold where they are on startup; nothing moves until user input.
"Follow" on the Mobile IO aligns the follower to the leader, then follows.
Input loss holds the follower and resumes in Idle.

Threads: the follower arm's thread runs the state machine. The leader arm runs
its own thread; its update callback applies haptic feedback. Lock order is
follower -> leader only: the leader's callback never takes the follower's
lock, it reads a snapshot of follower positions instead.

    python -m kits.arms.ex_leader_follower ex_leader_follower_leader.cfg.yaml ex_leader_follower.cfg.yaml

Takes the leader arm's config, then the follower arm's. The follower config's
``mobile_io_family`` (user_data) names the Mobile IO.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import sleep
from typing import TYPE_CHECKING

import hebi
import numpy as np
from hebi.util import create_mobile_io

from kits.common.config import add_config_argument, config_arg_parser, load_config, required_user_data
from kits.common.state_machine import ControllerState

from .arm_controller import CONFIG_DIR, ArmController, ArmExitState, ArmFaultState

if TYPE_CHECKING:
    from collections.abc import Sequence

    from hebi._internal.mobile_io import MobileIO
    from hebi.arm import Arm
    from numpy.typing import NDArray


DEFAULT_HAPTIC_GAINS = np.array([30, 25, 20, 10, 3, 2]) * 10.0


@dataclass
class LeaderFollowerInputs:
    home: bool = False
    follow: bool = False
    haptic_fbk: bool = False


class _ResumeToIdle:
    controller: LeaderFollowerController

    def resume_state(self):
        return LFIdleState(self.controller)


@dataclass(eq=False)
class LFStartupState(ControllerState["LeaderFollowerController"]):
    # Never move on startup: hold both arms where they are (Idle holds the follower).
    def update(self, t_now, **kwargs):
        c = self.controller
        with c.leader.lock:
            c.hold_position(manager=c.leader)
        return LFIdleState(c)


@dataclass(eq=False)
class LFHomingState(_ResumeToIdle, ControllerState["LeaderFollowerController"]):
    """Drive both leader and follower to the home pose."""

    def on_enter(self, from_state, **kwargs):
        c = self.controller
        c.move_to(c.arm_home, c.homing_time)
        with c.leader.lock:
            goal = hebi.arm.Goal(c.leader.size)
            goal.add_waypoint(t=c.homing_time, position=c.arm_home)
            c.leader.set_goal(goal)

    def update(self, t_now, **kwargs):
        if self.controller.arm.at_goal:
            return LFIdleState(self.controller)
        return None


@dataclass(eq=False)
class LFIdleState(_ResumeToIdle, ControllerState["LeaderFollowerController"]):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state, **kwargs):
        self.controller.hold_position()

    def receive_inputs(self, t_now, inputs: LeaderFollowerInputs, **kwargs):
        if self.requested is not None:
            return
        if inputs.home:
            self.requested = LFHomingState(self.controller)
        elif inputs.follow:
            self.requested = LFAligningState(self.controller)

    def update(self, t_now, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class LFAligningState(_ResumeToIdle, ControllerState["LeaderFollowerController"]):
    """Move the follower to the leader's current pose before following."""

    goal_set: bool = False
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def update(self, t_now, **kwargs):
        if self.requested is not None:
            requested, self.requested = self.requested, None
            return requested
        c = self.controller
        if not self.goal_set:
            # Set the goal here, not in on_enter: on_enter may not change state.
            if c.leader_position is None:
                print(f"{c.prefix}No leader feedback yet, staying idle")
                return LFIdleState(c)
            c.move_to(c.leader_position, c.homing_time)
            self.goal_set = True
        elif c.arm.at_goal:
            return LFFollowState(c)
        return None

    def receive_inputs(self, t_now, inputs: LeaderFollowerInputs, **kwargs):
        if self.requested is not None:
            return
        if inputs.home:
            self.requested = LFHomingState(self.controller)
        elif not inputs.follow:
            self.requested = LFIdleState(self.controller)


@dataclass(eq=False)
class LFFollowState(_ResumeToIdle, ControllerState["LeaderFollowerController"]):
    last_good_position: NDArray[np.float64] | None = None
    last_good_velocity: NDArray[np.float64] | None = None
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state, **kwargs):
        c = self.controller
        # Leader goes compliant to serve as the input device; the follower is
        # commanded directly from leader feedback in update().
        with c.leader.lock:
            c.leader.cancel_goal()
        c.arm.cancel_goal()

    def update(self, t_now, **kwargs):
        if self.requested is not None:
            requested, self.requested = self.requested, None
            return requested
        c = self.controller
        pos, vel = c.leader_position, c.leader_velocity
        if pos is not None and not np.any(np.isnan(pos)):
            self.last_good_position, self.last_good_velocity = pos, vel
        elif self.last_good_position is not None:
            # Bad leader sample: hold the last good pose rather than command NaN (limp).
            pos, vel = self.last_good_position, np.zeros_like(self.last_good_position)
        else:
            # No good leader sample yet: hold where the follower is.
            pos, vel = c.current_position(), np.zeros(c.arm.size)
        c.arm.pending_command.position = pos
        c.arm.pending_command.velocity = vel
        # The mirror plugin already ran this tick with the old command; re-run it.
        if mirror := c.arm.get_plugin_by_type(hebi.arm.DoubledJointMirror):
            mirror.update(c.arm, 0)
        return None

    def receive_inputs(self, t_now, inputs: LeaderFollowerInputs, **kwargs):
        if self.requested is not None:
            return
        if inputs.home:
            self.requested = LFHomingState(self.controller)
        elif not inputs.follow:
            self.requested = LFIdleState(self.controller)
        else:
            self.controller.haptic_enabled = inputs.haptic_fbk

    def on_exit(self, to_state, **kwargs):
        self.controller.haptic_enabled = False
        self.controller.hold_position()


class LeaderFollowerController(ArmController[LeaderFollowerInputs]):
    """State machine on the follower arm; the leader arm runs its own thread.

    ``start()`` / ``stop()`` start and stop both arms.
    """

    def __init__(
        self,
        follower: Arm,
        leader: Arm,
        home_position: Sequence[float] | NDArray[np.float64],
        *,
        homing_time: float = 3.0,
        haptic_gains: Sequence[float] | NDArray[np.float64] = DEFAULT_HAPTIC_GAINS,
        haptic_limit: float = 100,
        name: str = "Follower",
        input_timeout: float | None = 1.0,
    ):
        super().__init__(follower, LFStartupState, ArmFaultState, ArmExitState, name=name, input_timeout=input_timeout)
        self.leader = leader
        self.arm_home = np.asarray(home_position, dtype=np.float64)
        self.homing_time = homing_time
        self.haptic_gains = np.asarray(haptic_gains, dtype=np.float64)
        self.haptic_limit = haptic_limit
        self.haptic_enabled = False

        # Snapshots exchanged between the two arm threads (attribute swaps are atomic).
        self.leader_position: NDArray[np.float64] | None = None
        self.leader_velocity: NDArray[np.float64] | None = None
        self._follower_position: NDArray[np.float64] | None = None
        self.add_update_handler(self._snapshot_follower)

    def start(self):
        self.leader.set_on_update_callback(self._on_leader_update)
        self.leader.start()
        try:
            super().start()
        except Exception:
            self.leader.stop()
            raise

    def stop(self):
        super().stop()
        if self.leader.running:
            self.leader.stop()

    def _snapshot_follower(self, controller, state) -> None:
        # Update handler (normal states only); haptics only run in Follow.
        self._follower_position = np.array(self.arm.last_feedback.position, dtype=np.float64)

    def _on_leader_update(self) -> bool:
        # Leader thread, leader lock held. Never take the follower's lock here.
        fbk = self.leader.last_feedback
        self.leader_position = np.array(fbk.position, dtype=np.float64)
        self.leader_velocity = np.array(fbk.velocity, dtype=np.float64)

        follower_pos = self._follower_position
        if self.haptic_enabled and follower_pos is not None:
            pos_diff = self.leader_position - follower_pos
            haptic_effort = np.clip(self.haptic_gains * pos_diff * np.abs(pos_diff), -self.haptic_limit, self.haptic_limit)
            if not np.any(np.isnan(haptic_effort)):
                self.leader.pending_command.effort = self.leader.pending_command.effort - haptic_effort
        return True


def setup_mobile_io(m: MobileIO):
    m.resetUI()
    m.set_button_label(1, "⟲")

    m.set_button_label(2, "Follow")
    m.set_button_mode(2, 1)

    m.set_button_label(3, "Haptic")
    m.set_button_mode(3, 1)

    for i in range(4, 8):
        m.set_button_label(i, " ")
    m.set_button_label(8, "Quit")

    for i in range(1, 9):
        m.set_axis_label(i, " ")


def parse_mobile_feedback(m: MobileIO, timeout_ms: float = 100.0):
    if not m.update(timeout_ms):
        return None

    return LeaderFollowerInputs(
        home=bool(m.get_button_state(1)),
        follow=m.get_button_state(2) == 1,
        haptic_fbk=m.get_button_state(3) == 1,
    )


if __name__ == "__main__":
    parser = config_arg_parser("Leader/follower arms with haptic feedback", CONFIG_DIR)
    add_config_argument(parser, "leader_config", "Leader arm config, e.g. ex_leader_follower_leader.cfg.yaml")
    add_config_argument(parser, "follower_config", "Follower arm config, e.g. ex_leader_follower.cfg.yaml")
    args = parser.parse_args()
    leader_cfg = load_config(args.leader_config, CONFIG_DIR)
    cfg = load_config(args.follower_config, CONFIG_DIR)
    lookup = hebi.Lookup()
    sleep(2)

    # Find all hardware before anything starts moving
    leader_arm = hebi.arm.create_from_config(leader_cfg, lookup)
    follower_arm = hebi.arm.create_from_config(cfg, lookup)
    mio_family = required_user_data(cfg, "mobile_io_family")

    control = LeaderFollowerController(follower_arm, leader_arm, home_position=required_user_data(cfg, "home_position"))

    print("Looking for Mobile IO...")
    m = create_mobile_io(lookup, mio_family)
    while m is None:
        try:
            print("Waiting for Mobile IO device to come online...")
            sleep(1)
            m = create_mobile_io(lookup, mio_family)
        except KeyboardInterrupt:
            raise SystemExit(0) from None

    m.update()
    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    control.start()
    try:
        while control.running:
            inputs = parse_mobile_feedback(m)
            if m.get_button_state(8):
                break
            if inputs is not None:
                control.receive_inputs(inputs)
    except KeyboardInterrupt:
        pass
    finally:
        control.stop()
        m.set_led_color("red")
