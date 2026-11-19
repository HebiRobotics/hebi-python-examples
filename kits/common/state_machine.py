"""Shared state machine base for HEBI kit controllers.

A :class:`Controller` wraps a :class:`hebi.arm.GroupManager` (``TreadedBase``,
``hebi.arm.Arm``, ``CameraMast``, ...) and runs a state machine on the
manager's control thread:

* ``Controller.start()`` starts the manager's thread. Nothing runs before
  that, so all hardware lookup and handler registration can happen first.
* Each control tick (under ``manager.lock``) the controller checks, in order:
  exit state -> M-stop (fault state) -> input timeout (disconnected state),
  then ``Controller.pre_update()``, then ``state.update(t_now=...)``, then every
  update handler unless disconnected (no inputs means nothing to update from).
* When the M-stop is released, the controller resumes into
  ``resume_state()`` (normally an idle state). Faults from errors in the
  control loop latch.
* ``Controller.receive_inputs()`` is called from the main thread and is
  serialized with the control tick by taking ``manager.lock``.
* ``Controller.stop()`` must be called from the main thread: it transitions
  to the exit state and joins the manager's thread.

States subclass :class:`ControllerState` and override any of ``on_enter``,
``update``, ``receive_inputs`` and ``on_exit``.

Transitions are declarative: **only** ``update()`` decides to change state, by
*returning* the new state (``return NewState(self.controller)``). Returning
``None`` stays in the current state.

``receive_inputs()`` never returns a state and never transitions. If an input
should cause a transition, it records the request in a field on the state and
the next ``update()`` returns the intended state, so an input takes effect on
the next control tick, not inside ``receive_inputs``.

The only other transitions are the ones the Controller triggers itself: fault
(M-stop or an error in the control loop), input-timeout disconnect, resuming
after a disconnect or a recovered fault, and exit on ``stop()``.
"""

from __future__ import annotations

import datetime
import time
import traceback
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Generic, TypeVar

import hebi
import numpy as np

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence
    from typing import TypeAlias

    from hebi._internal.group import Group
    from hebi._internal.lookup import Lookup
    from hebi.arm import GroupManager


C = TypeVar("C", bound="Controller[Any, Any]")
M = TypeVar("M", bound="GroupManager")
I = TypeVar("I")


@dataclass(eq=False)
class ControllerState(Generic[C]):
    """One state of a :class:`Controller`.

    Every hook runs with ``controller.manager.lock`` held. ``update`` and the
    timeout/fault checks run on the manager's control thread, while
    ``receive_inputs`` runs on whichever thread called
    ``Controller.receive_inputs`` (normally the main thread).

    Hooks must never block (no network round trips or sleeps), because they
    stall the control loop.
    """

    controller: C

    @property
    def name(self) -> str:
        return type(self).__name__

    def can_transition_to(self, state: ControllerState[C]) -> bool:
        return True

    def can_transition_from(self, state: ControllerState[C]) -> bool:
        return True

    def on_enter(self, from_state: ControllerState[C] | None, **kwargs) -> None: ...

    def update(self, t_now: float, **kwargs) -> ControllerState[C] | None:
        """Run one control tick. Return the next state to transition to, or ``None`` to stay."""
        return None

    def receive_inputs(self, t_now: float, inputs: Any, **kwargs) -> None:
        """Record one input sample. Never transitions: store a request on ``self`` for ``update()`` to act on."""

    def on_exit(self, to_state: ControllerState[C], **kwargs) -> None: ...

    def resume_state(self) -> ControllerState[C] | None:
        """State to resume into if this state is interrupted by a disconnect or fault.

        Return a *new* state instance, or ``None`` to defer to
        ``Controller.resume_state``. Motion states (homing, aligning, ...)
        usually resume to an idle/teleop state rather than restarting motion.
        """
        return None


@dataclass(eq=False)
class DisconnectedState(ControllerState["Controller[Any, Any]"]):
    """Entered when no inputs have arrived for ``controller.input_timeout``.

    On entry, calls ``controller.on_disconnect()``, which holds position by
    default. The next input marks it reconnected, and the following ``update``
    moves to ``controller.resume_state(prev_state)``. The interrupted state
    instance is never re-entered, so its ``on_enter`` side effects (homing,
    alignment, ...) don't run a second time.
    """

    prev_state: ControllerState[Any] | None = field(default=None, init=False)
    reconnected: bool = field(default=False, init=False)

    def on_enter(self, from_state, **kwargs):
        self.prev_state = from_state
        self.reconnected = False
        print(f"{self.controller.prefix}Input timeout, holding position.")
        self.controller.on_disconnect()

    def receive_inputs(self, t_now, inputs, **kwargs):
        self.reconnected = True

    def update(self, t_now, **kwargs):
        if self.reconnected:
            print(f"{self.controller.prefix}Inputs reconnected, resuming.")
            return self.controller.resume_state(self.prev_state)
        return None


if TYPE_CHECKING:
    TransitionHandler: TypeAlias = "Callable[[Controller[Any, Any], ControllerState[Any]], None]"
    UpdateHandler: TypeAlias = "Callable[[Controller[Any, Any], ControllerState[Any]], None]"


class Controller(Generic[M, I]):
    """State machine driven by a ``GroupManager``'s control thread.

    :param manager: The group manager this controller drives. It must not be
        started yet; ``start()`` starts it.
    :param starting_state: State class to construct and enter on ``start()``.
    :param fault_state: State class entered when any module reports an M-stop.
    :param exit_state: State class entered by ``stop()``. Its ``on_enter`` must
        not call ``manager.stop()`` (that would join the control thread from
        itself); ``Controller.stop()`` does that after the transition.
    :param disconnected_state: State class entered on input timeout.
    :param input_timeout: Seconds without ``receive_inputs`` before moving to
        ``disconnected_state``. ``None`` disables the timeout.
    :param fault_recovery: If ``True`` (default), leave the fault state for
        ``resume_state()`` once the M-stop is released. If ``False``, an M-stop
        latches until the program restarts. Faults caused by errors in the
        control loop always latch. Fault states should drop motion targets
        (e.g. ``cancel_goal()``) rather than hold: modules ignore commands while
        M-stopped, and a stale target would be driven to on release.
    :param name: Prefix for log messages, e.g. ``"Base"`` -> ``"[Base] ..."``.
    """

    def __init__(
        self,
        manager: M,
        starting_state: type[ControllerState[Any]],
        fault_state: type[ControllerState[Any]],
        exit_state: type[ControllerState[Any]],
        disconnected_state: type[ControllerState[Any]] = DisconnectedState,
        *,
        input_timeout: float | None = 1.0,
        fault_recovery: bool = True,
        name: str = "",
    ):
        self.manager = manager
        self.starting_state = starting_state
        self.fault_state = fault_state
        self.exit_state = exit_state
        self.disconnected_state = disconnected_state
        self.input_timeout = input_timeout
        self.fault_recovery = fault_recovery
        self.name = name

        self.state: ControllerState[Any] = starting_state(self)
        self.last_input_t = time.time()

        self._started = False
        self._transitioning = False
        self._pre_fault_state: ControllerState[Any] | None = None
        self._fault_from_mstop = False
        self._transition_handlers: list[TransitionHandler] = []
        self._update_handlers: list[UpdateHandler] = []
        self._last_packet_loss_print = 0.0

    # ------------------------------------------------------------------
    # Setup and lifecycle (main thread)
    # ------------------------------------------------------------------

    @property
    def prefix(self) -> str:
        return f"[{self.name}] " if self.name else ""

    def add_transition_handler(self, handler: TransitionHandler) -> None:
        """Call ``handler(controller, new_state)`` on every transition.

        Handlers run on the control thread, after ``new_state.on_enter`` and
        before ``controller.state`` is updated, so ``controller.state`` is
        still the previous state. Handlers must not block. Register them
        before ``start()`` so they also see the starting state.
        """
        self._transition_handlers.append(handler)

    def add_update_handler(self, handler: UpdateHandler) -> None:
        """Call ``handler(controller, state)`` after every ``state.update``.

        Not called in the disconnected, fault or exit states.
        """
        self._update_handlers.append(handler)

    def start(self) -> None:
        """Enter the starting state and start the manager's control thread."""
        if self._started:
            raise RuntimeError(f"{self.prefix}Controller already started")
        if self.manager.running:
            raise RuntimeError(f"{self.prefix}Manager thread is already running; let Controller.start() start it")

        self.manager.set_on_update_callback(self._on_manager_update)
        self.manager.set_on_failed_update_callback(self._on_packet_loss)

        with self.manager.lock:
            self.state.on_enter(from_state=None)
            for handler in self._transition_handlers:
                handler(self, self.state)

        self.last_input_t = time.time()
        self._started = True
        self.manager.start()

    def stop(self) -> None:
        """Transition to the exit state and stop the control thread.

        Must be called from the main thread, never from a state hook or handler.
        """
        with self.manager.lock:
            self._transition_to(self.exit_state(self))
        if self.manager.running:
            self.manager.stop()

    @property
    def running(self) -> bool:
        if isinstance(self.state, self.exit_state):
            return False
        # The control thread may have died (e.g. an uncaught error in the manager).
        return not self._started or self.manager.running

    def start_logging(self, directory: str, prefix: str) -> None:
        """Start logging the manager's group to ``directory/<prefix>_<timestamp>``."""
        self.manager.start_log(directory, timestamped_log_name(prefix), mkdirs=True)

    # ------------------------------------------------------------------
    # Inputs and transitions
    # ------------------------------------------------------------------

    def receive_inputs(self, inputs: I) -> None:
        """Forward one input sample to the current state. Thread-safe."""
        with self.manager.lock:
            t_now = time.time()
            self.last_input_t = t_now
            self.state.receive_inputs(t_now, inputs)

    def _transition_to(self, new_state: ControllerState[Any], **kwargs) -> bool:
        """Move to ``new_state``. Thread-safe.

        Internal to the Controller (fault, disconnect, resume, exit, and
        applying the state returned by ``update``). User code and states must
        never call this: return the new state from ``update`` instead.

        Returns ``True`` if the controller ends up in a state of
        ``new_state``'s type (including when it already was), ``False`` if a
        guard refused the transition.
        """
        with self.manager.lock:
            if self._transitioning:
                raise RuntimeError(
                    f"{self.prefix}_transition_to({new_state.name}) called during another transition; "
                    "on_enter/on_exit and transition handlers must not change state"
                )
            if not new_state.can_transition_from(self.state):
                return False
            if not self.state.can_transition_to(new_state):
                return False
            if type(self.state) is type(new_state):
                return True

            print(f"{self.prefix}{self.state.name} -> {new_state.name}")
            self._transitioning = True
            try:
                self.state.on_exit(to_state=new_state, **kwargs)
                new_state.on_enter(from_state=self.state, **kwargs)
                for handler in self._transition_handlers:
                    handler(self, new_state)
                self.state = new_state
            finally:
                self._transitioning = False
            return True

    # ------------------------------------------------------------------
    # Hooks for subclasses
    # ------------------------------------------------------------------

    def resume_state(self, prev_state: ControllerState[Any] | None) -> ControllerState[Any]:
        """State to move to after a disconnect or a recovered fault.

        Asks ``prev_state.resume_state()`` first. If that returns ``None`` (or
        there is no previous state), restarts from ``starting_state``.
        """
        if prev_state is not None and (state := prev_state.resume_state()) is not None:
            return state
        return self.starting_state(self)

    def on_disconnect(self) -> None:
        """Called when entering ``DisconnectedState``. Holds position by default."""
        self.hold_position()

    def pre_update(self, t_now: float) -> ControllerState[Any] | None:
        """Hook run every normal tick before ``state.update``.

        Return a state to transition to it (and skip this tick's
        ``state.update``), or ``None`` to carry on. Use it for conditions that
        can pre-empt *any* state, such as a latching fault monitor. Not called
        in the exit, fault or disconnected-by-timeout paths.
        """
        return None

    def hold_position(self, ramp_time: float = 0.25, manager: GroupManager | None = None) -> None:
        """Ramp a manager's trajectory (default: this controller's) to a stop at its current position.

        Needs feedback, so call it from the control thread, not from ``on_enter``
        of the starting state (which runs in ``start()`` before any feedback).
        """
        manager = self.manager if manager is None else manager
        pos, _, _ = manager.current_state()
        pos = np.array(pos, dtype=np.float64)
        # A velocity-only trajectory (e.g. joystick jogging) has NaN positions,
        # which would make this a zero-velocity command instead of a hold.
        fbk = manager.last_feedback
        for fallback in (fbk.position_command, fbk.position):
            nan = np.isnan(pos)
            if not nan.any():
                break
            pos[nan] = np.asarray(fallback)[nan]
        goal = hebi.Goal(manager.size)
        goal.add_waypoint(t=ramp_time, position=pos, velocity=np.zeros(manager.size))
        manager.set_goal(goal)

    # ------------------------------------------------------------------
    # Control thread
    # ------------------------------------------------------------------

    def _on_manager_update(self) -> bool:
        # Called by GroupManager._inner_update with manager.lock held.
        # Always returns True: returning False makes the manager report a
        # failed update, and our own errors are handled here instead.
        t_now = time.time()
        try:
            self._tick(t_now)
        except Exception:  # noqa: BLE001 - thread boundary: an error in a state must not kill the control thread
            print(f"{self.prefix}Error in control loop, entering {self.fault_state.__name__}:")
            traceback.print_exc()
            if not isinstance(self.state, (self.fault_state, self.exit_state)):
                try:
                    self._enter_fault()
                except Exception:  # noqa: BLE001 - same boundary, fault state's own on_enter failed
                    traceback.print_exc()
        return True

    def _tick(self, t_now: float) -> None:
        state = self.state

        if isinstance(state, self.exit_state):
            return

        if self.manager.m_stopped:
            if not isinstance(state, self.fault_state):
                print(f"{self.prefix}M-stop pressed")
                self._enter_fault(from_mstop=True)
            return

        if isinstance(state, self.fault_state):
            if self.fault_recovery and self._fault_from_mstop:
                print(f"{self.prefix}M-stop released")
                self._transition_to(self.resume_state(self._pre_fault_state))
            return

        if (
            self.input_timeout is not None
            and not isinstance(state, self.disconnected_state)
            and t_now - self.last_input_t > self.input_timeout
        ):
            self._transition_to(self.disconnected_state(self))
            return

        next_state = self.pre_update(t_now)
        if next_state is None:
            next_state = self.state.update(t_now=t_now)
        if next_state is not None:
            self._transition_to(next_state)
        if isinstance(self.state, self.disconnected_state):
            return
        for handler in self._update_handlers:
            handler(self, self.state)

    def _enter_fault(self, from_mstop: bool = False) -> None:
        self._fault_from_mstop = from_mstop
        prev = self.state
        # Don't resume into the disconnected state; resume from what it interrupted.
        if isinstance(prev, DisconnectedState):
            prev = prev.prev_state
        self._pre_fault_state = prev
        self._transition_to(self.fault_state(self))

    def _on_packet_loss(self) -> None:
        t_now = time.time()
        if t_now - self._last_packet_loss_print > 1.0:
            self._last_packet_loss_print = t_now
            print(f"{self.prefix}WARNING: Failed to get feedback")


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------


def timestamped_log_name(prefix: str) -> str:
    """``"<prefix>_YYYY-MM-DD_HH-MM-SS"``: no ``:``, so it's valid on every OS."""
    return f"{prefix}_{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}"


def wait_for_group(
    lookup: Lookup,
    families: str | Sequence[str],
    names: Sequence[str],
    timeout_ms: int = 1000,
    attempts: int | None = None,
) -> Group:
    """Block until the modules are found on the network.

    :param attempts: Give up and raise ``RuntimeError`` after this many tries.
        ``None`` waits forever.
    """
    if isinstance(families, str):
        families = [families]
    tries = 0
    while (group := lookup.get_group_from_names(list(families), list(names), timeout_ms=timeout_ms)) is None:
        tries += 1
        modules = ", ".join(f"{families[0] if len(families) == 1 else families[i]}/{n}" for i, n in enumerate(names))
        if attempts is not None and tries >= attempts:
            raise RuntimeError(f"Could not find modules: {modules}")
        print(f"Looking for modules: {modules}")
    return group
