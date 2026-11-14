"""Threaded treaded base (kits/tready/treaded_base*.py) against an imitated Tready, with the real MobileIOUpdater."""

import time
from types import SimpleNamespace

import hebi
import numpy as np
import pytest

from conftest import REPO_ROOT, unique_family

CONFIG_DIR = REPO_ROOT / "kits" / "tready" / "config"


@pytest.fixture
def tready(imitation, fake_mobile_io, monkeypatch):
    """A TreadedBaseController wired to a MobileIOUpdater, not started. Yields (controller, updater, fake mobile io)."""
    from kits.tready import treaded_base as tb
    from kits.tready.mobile_io_manager import MobileIOUpdater, update_drive_mode, update_startup_mode
    from kits.tready.treaded_base_core import TreadedBase, TreadedBaseController, TreadyEStopState, TreadyExitState

    cfg = hebi.config.load_config(str(CONFIG_DIR / "tready.cfg.yaml"))
    family = unique_family("tready")
    cfg.families = [family]
    imitation([(family, name) for name in cfg.names])

    # Imitated modules report the M-stop as pressed; drive it from the test instead.
    monkeypatch.setattr(TreadedBase, "m_stopped", property(lambda self: getattr(self, "test_mstop", False)))
    base = TreadedBase.from_config(cfg)
    ctrl = TreadedBaseController(base, tb.TreadyInitialState, TreadyEStopState, TreadyExitState, max_base_speed=0.25)

    updater = MobileIOUpdater(
        fake_mobile_io,
        {
            tb.TreadyStartupState: (CONFIG_DIR / "layouts" / "TreadwardStartupController.json", update_startup_mode),
            tb.TreadyTeleopState: (CONFIG_DIR / "layouts" / "TreadwardDriveController.json", update_drive_mode),
        },
    )
    ctrl._transition_handlers.append(updater.base_transition_handler)
    ctrl._update_handlers.append(updater.update_voltage_reading)
    ctrl._update_handlers.append(updater.update_torque_mode)
    ctrl._update_handlers.append(updater.update_startup_msg_base)
    yield ctrl, updater, fake_mobile_io
    if base.running:
        base.stop()


def pump(ctrl, updater, m, seconds, feed=True):
    """Main-loop stand-in: poll the updater and forward base inputs (feed=False simulates a Mobile IO dropout)."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        if feed:
            inputs = updater.parse_mobile_io_feedback(m)
            if inputs is not None:
                ctrl.receive_inputs(inputs[0])
        time.sleep(0.02)


def wait_for(ctrl, updater, m, cls, timeout=5.0, feed=True):
    deadline = time.time() + timeout
    while time.time() < deadline and not isinstance(ctrl.state, cls):
        pump(ctrl, updater, m, 0.05, feed)
    return isinstance(ctrl.state, cls)


def test_startup_layout_is_sent(tready):
    """B1: the base thread started in the constructor, before handlers were registered, so the
    Initial -> Startup transition was missed and the startup layout never sent (demo stuck)."""
    from kits.tready import treaded_base as tb

    ctrl, updater, m = tready
    assert not ctrl.base.running, "constructor must not start the control thread"
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState), ctrl.state
    assert any(name == "send_layout" for name, _ in m.calls), "startup layout never sent"


def test_mobile_io_only_used_from_main_thread(tready):
    """B3: handlers on the base thread called the Mobile IO directly (including a blocking send_layout)."""
    from kits.tready import treaded_base as tb

    ctrl, updater, m = tready
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)
    pump(ctrl, updater, m, 0.3)
    assert m.calls, "test precondition: the updater used the Mobile IO"
    assert set(m.call_threads) == {"MainThread"}, set(m.call_threads)


def test_reconnect_does_not_restart_homing(tready, monkeypatch):
    """B5: reconnecting re-ran the interrupted state's on_enter, restarting the homing move."""
    from kits.tready import treaded_base as tb
    from kits.tready.treaded_base_core import TreadedBaseController, TreadyDisconnectedState

    ctrl, updater, m = tready
    homes = []
    original_home = ctrl.base.home
    monkeypatch.setattr(ctrl.base, "home", lambda *a, **k: (homes.append(1), original_home(*a, **k)))
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)

    m.press(b1=1)  # home
    assert wait_for(ctrl, updater, m, tb.TreadyHomingState, timeout=2)
    m.press(b1=0)
    assert wait_for(ctrl, updater, m, TreadyDisconnectedState, timeout=3, feed=False)
    pump(ctrl, updater, m, 0.3)
    assert not isinstance(ctrl.state, TreadyDisconnectedState)
    assert len(homes) == 1, f"homing started {len(homes)} times"


def test_estop_drops_targets(tready):
    """B6: E-stop held the flipper position from when it was pressed; modules drive back to it on release."""
    from kits.tready import treaded_base as tb
    from kits.tready.treaded_base_core import TreadyEStopState

    ctrl, updater, m = tready
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)
    ctrl.base.test_mstop = True
    assert wait_for(ctrl, updater, m, TreadyEStopState, timeout=2)
    assert ctrl.base.flipper_traj is None and ctrl.base.chassis_traj is None


def test_disconnect_stops_the_base(tready):
    """A9: with no Mobile IO input the last wheel velocity kept being sent, so the base kept driving."""
    from kits.tready import treaded_base as tb
    from kits.tready.treaded_base_core import TreadyDisconnectedState

    ctrl, updater, m = tready
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)
    m.axes[2] = 1.0  # drive forward (left pad up/down)
    pump(ctrl, updater, m, 1.0)
    with ctrl.base.lock:
        assert np.any(np.abs(ctrl.base.wheel_cmd.velocity) > 0.1), "test precondition: base is driving"

    assert wait_for(ctrl, updater, m, TreadyDisconnectedState, timeout=3, feed=False)
    pump(ctrl, updater, m, 0.5, feed=False)
    with ctrl.base.lock:
        assert np.allclose(ctrl.base.wheel_cmd.velocity, 0.0)
        assert np.allclose(ctrl.base.flipper_cmd.velocity, 0.0)


def test_estop_released_resumes_teleop(tready):
    """Releasing the M-stop returns to the state the E-stop interrupted, without any Mobile IO reconnect."""
    from kits.tready import treaded_base as tb
    from kits.tready.treaded_base_core import TreadyEStopState

    ctrl, updater, m = tready
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)
    ctrl.base.test_mstop = True
    assert wait_for(ctrl, updater, m, TreadyEStopState, timeout=2)

    ctrl.base.test_mstop = False
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState, timeout=2), ctrl.state


def test_mstop_during_disconnect_recovers(tready):
    """A10: ESTOP and DISCONNECTED handed control back and forth forever, never returning to the interrupted state."""
    from kits.tready import treaded_base as tb
    from kits.tready.treaded_base_core import TreadyDisconnectedState, TreadyEStopState

    ctrl, updater, m = tready
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)
    assert wait_for(ctrl, updater, m, TreadyDisconnectedState, timeout=3, feed=False)

    ctrl.base.test_mstop = True
    assert wait_for(ctrl, updater, m, TreadyEStopState, timeout=2, feed=False)

    ctrl.base.test_mstop = False
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState, timeout=3), ctrl.state


def test_compliant_mode_labels_are_sent(tready):
    """B7: compliant mode's update handler read torque_labels_changed before any input set it (fault), and
    two inputs between control ticks could hide a label change."""
    from kits.tready import treaded_base as tb

    ctrl, updater, m = tready
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)
    m.press(b2=1)  # torque mode
    assert wait_for(ctrl, updater, m, tb.TreadyCompliantFlippersState, timeout=2), ctrl.state
    pump(ctrl, updater, m, 0.3)
    assert isinstance(ctrl.state, tb.TreadyCompliantFlippersState), "compliant state faulted"
    assert any(name == "set_axis_label" and "Max" in str(args) for name, args in m.calls)


def test_stow_stops_driving(tready):
    """B10: Deployed applied the stow sample's joystick velocity, and Stowing never updates the chassis."""
    from kits.tready import treaded_base as tb
    from kits.tready.treaded_base_core import ChassisVelocity, TreadyInputs

    ctrl, updater, m = tready
    ctrl.start()
    assert wait_for(ctrl, updater, m, tb.TreadyTeleopState)
    with ctrl.base.lock:
        ctrl.transition_to(tb.TreadyDeployedState(ctrl))
    ctrl.receive_inputs(TreadyInputs(stow=True, stow_safe=False, base_motion=ChassisVelocity(1.0, 0.0)))
    assert isinstance(ctrl.state, tb.TreadyStowingState)
    velocity = ctrl.base.chassis_traj.get_state(ctrl.base.chassis_traj.end_time)[1]
    assert np.allclose(velocity, 0.0), velocity


def test_base_pose_with_nan_orientation():
    """A NaN IMU quaternion (no orientation yet) made Rotation.from_quat raise in torque mode."""
    from kits.tready.treaded_base_core import TreadedBase

    stub = SimpleNamespace(
        last_feedback=SimpleNamespace(position=np.zeros(8), orientation=np.full((8, 4), np.nan)),
        robot_model=hebi.robot_model.import_from_hrdf(str(CONFIG_DIR / "hrdf" / "Tready.hrdf")),
    )
    assert np.all(np.isfinite(TreadedBase.pose.fget(stub)))


def test_updater_idle_inputs_before_a_layout_is_active(fake_mobile_io):
    """No parser yet but the Mobile IO is sending: return the idle inputs, not None (None means a disconnect)."""
    from kits.tready.mobile_io_manager import MobileIOUpdater

    idle = (None, object())
    assert MobileIOUpdater(fake_mobile_io, {}, idle_inputs=idle).parse_mobile_io_feedback(fake_mobile_io) is idle
    assert MobileIOUpdater(fake_mobile_io, {}).parse_mobile_io_feedback(fake_mobile_io) is None
