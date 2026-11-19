"""kits.common.state_machine: Controller/ControllerState behavior with a fake manager (no hardware, no threads).

The controller is ticked by hand with ``ctrl.tick()``, so these tests are deterministic and can run together
(unlike the ``hebi imitation`` tests).
"""

import threading
import time
from dataclasses import dataclass, field

import pytest

from kits.common.state_machine import Controller, ControllerState, DisconnectedState


class FakeManager:
    """The parts of hebi.arm.GroupManager that Controller uses."""

    def __init__(self):
        self.lock = threading.RLock()
        self.running = False
        self.m_stopped = False

    def set_on_update_callback(self, cb):
        self.on_update = cb

    def set_on_failed_update_callback(self, cb):
        self.on_failed_update = cb

    def start(self):
        self.running = True

    def stop(self):
        self.running = False


@dataclass(eq=False)
class IdleState(ControllerState):
    go: bool = field(default=False, init=False)
    updates: int = field(default=0, init=False)

    def receive_inputs(self, t_now, inputs, **kwargs):
        # Inputs only record a request; update() decides the transition.
        self.go = inputs == "go"

    def update(self, t_now, **kwargs):
        self.updates += 1
        if self.go:
            return RunningState(self.controller)
        return None

    def resume_state(self):
        return IdleState(self.controller)


@dataclass(eq=False)
class RunningState(ControllerState):
    def update(self, t_now, **kwargs):
        return None


@dataclass(eq=False)
class MovingState(ControllerState):
    """A motion state: on interruption it resumes into Idle, not into itself."""

    def resume_state(self):
        return IdleState(self.controller)


@dataclass(eq=False)
class FaultState(ControllerState):
    pass


@dataclass(eq=False)
class ExitState(ControllerState):
    pass


class TestController(Controller):
    __test__ = False  # not a pytest class

    def __init__(self, manager, **kwargs):
        super().__init__(manager, IdleState, FaultState, ExitState, **kwargs)
        self.disconnects = 0
        self.faults_checked = []

    def on_disconnect(self):
        self.disconnects += 1  # the default holds position, which needs hardware

    def tick(self):
        assert self.manager.on_update()


@pytest.fixture
def ctrl():
    c = TestController(FakeManager(), input_timeout=1.0)
    c.start()
    return c


def test_update_return_applies_the_transition(ctrl):
    ctrl.receive_inputs("go")
    ctrl.tick()
    assert isinstance(ctrl.state, RunningState)


def test_receive_inputs_alone_never_changes_state(ctrl):
    ctrl.receive_inputs("go")
    assert isinstance(ctrl.state, IdleState), "an input takes effect on the next tick, not inside receive_inputs"
    ctrl.tick()
    assert isinstance(ctrl.state, RunningState)


def test_update_returning_none_stays(ctrl):
    ctrl.tick()
    ctrl.tick()
    assert isinstance(ctrl.state, IdleState)
    assert ctrl.state.updates == 2


def test_guard_refusing_the_transition_stays():
    @dataclass(eq=False)
    class Locked(IdleState):
        def can_transition_to(self, state):
            return not isinstance(state, RunningState)

    ctrl = TestController(FakeManager())
    ctrl.starting_state = Locked
    ctrl.state = Locked(ctrl)
    ctrl.start()
    ctrl.receive_inputs("go")
    ctrl.tick()
    assert isinstance(ctrl.state, Locked)


def test_transition_to_same_type_keeps_the_current_instance(ctrl):
    before = ctrl.state
    assert ctrl._transition_to(IdleState(ctrl)) is True
    assert ctrl.state is before


def test_nested_transition_from_on_enter_is_an_error(ctrl):
    @dataclass(eq=False)
    class Bad(ControllerState):
        def on_enter(self, from_state, **kwargs):
            self.controller._transition_to(RunningState(self.controller))

    with pytest.raises(RuntimeError, match="during another transition"):
        ctrl._transition_to(Bad(ctrl))
    assert isinstance(ctrl.state, IdleState)


def test_input_timeout_disconnects_and_resumes_on_the_next_tick(ctrl):
    ctrl.last_input_t = time.time() - 10
    ctrl.tick()
    assert isinstance(ctrl.state, DisconnectedState)
    assert ctrl.disconnects == 1

    ctrl.receive_inputs("anything")
    assert isinstance(ctrl.state, DisconnectedState), "reconnect is handled by update, not receive_inputs"
    ctrl.tick()
    assert isinstance(ctrl.state, IdleState)


def test_disconnect_resumes_a_motion_state_into_its_resume_state(ctrl):
    ctrl._transition_to(MovingState(ctrl))
    ctrl.last_input_t = time.time() - 10
    ctrl.tick()
    assert isinstance(ctrl.state, DisconnectedState)
    ctrl.receive_inputs("x")
    ctrl.tick()
    assert isinstance(ctrl.state, IdleState), "interrupted motion must not restart"


def test_input_timeout_can_be_disabled():
    ctrl = TestController(FakeManager(), input_timeout=None)
    ctrl.start()
    ctrl.last_input_t = time.time() - 100
    ctrl.tick()
    assert isinstance(ctrl.state, IdleState)


def test_update_handlers_are_skipped_while_disconnected(ctrl):
    calls = []
    ctrl.add_update_handler(lambda c, s: calls.append(type(s).__name__))
    ctrl.tick()
    assert calls == ["IdleState"]
    ctrl.last_input_t = time.time() - 10
    ctrl.tick()
    assert calls == ["IdleState"]


def test_mstop_enters_fault_and_recovers_when_released(ctrl):
    ctrl._transition_to(MovingState(ctrl))
    ctrl.manager.m_stopped = True
    ctrl.tick()
    assert isinstance(ctrl.state, FaultState)

    ctrl.manager.m_stopped = False
    ctrl.tick()
    assert isinstance(ctrl.state, IdleState)


def test_mstop_during_disconnect_resumes_the_interrupted_state_not_disconnected(ctrl):
    ctrl._transition_to(MovingState(ctrl))
    ctrl.last_input_t = time.time() - 10
    ctrl.tick()
    assert isinstance(ctrl.state, DisconnectedState)

    ctrl.manager.m_stopped = True
    ctrl.tick()
    assert isinstance(ctrl.state, FaultState)
    ctrl.manager.m_stopped = False
    ctrl.tick()
    assert isinstance(ctrl.state, IdleState)


def test_fault_recovery_can_be_disabled():
    ctrl = TestController(FakeManager(), fault_recovery=False)
    ctrl.start()
    ctrl.manager.m_stopped = True
    ctrl.tick()
    ctrl.manager.m_stopped = False
    ctrl.tick()
    assert isinstance(ctrl.state, FaultState)


def test_error_in_update_latches_the_fault_state(ctrl, capsys):
    @dataclass(eq=False)
    class Boom(ControllerState):
        def update(self, t_now, **kwargs):
            raise ValueError("boom")

    ctrl._transition_to(Boom(ctrl))
    ctrl.tick()
    assert isinstance(ctrl.state, FaultState)
    ctrl.tick()
    ctrl.tick()
    assert isinstance(ctrl.state, FaultState), "errors in the control loop latch"
    assert "boom" in capsys.readouterr().err


def test_pre_update_can_preempt_any_state_and_skips_update(ctrl):
    class Preempting(TestController):
        def pre_update(self, t_now):
            return MovingState(self) if self.trip else None

    c = Preempting(FakeManager())
    c.trip = False
    c.start()
    c.tick()
    idle = c.state
    assert idle.updates == 1

    c.trip = True
    c.tick()
    assert isinstance(c.state, MovingState)
    assert idle.updates == 1, "update() must not run on a tick where pre_update transitioned"


def test_stop_enters_the_exit_state_and_stops_the_manager(ctrl):
    assert ctrl.running
    ctrl.stop()
    assert isinstance(ctrl.state, ExitState)
    assert not ctrl.running
    assert not ctrl.manager.running
