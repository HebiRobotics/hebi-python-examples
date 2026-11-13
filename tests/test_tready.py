"""Treaded base controller (kits/tready/treaded_base_core.py) against an imitated Tready, plus Mobile IO parsing."""

import time
from types import SimpleNamespace

import numpy as np
import pytest

from conftest import REPO_ROOT, unique_family

TREADY_NAMES = [f"T{n + 1}_J2_track" for n in range(4)] + [f"T{n + 1}_J1_flipper" for n in range(4)]
CONFIG_DIR = REPO_ROOT / "kits" / "tready" / "config"


@pytest.fixture
def base_control(imitation, monkeypatch):
    """A TreadedBaseController on an imitated Tready. Its base runs the update thread; tests drive it with feed()."""
    from kits.tready.treaded_base import TreadyInitialState
    from kits.tready.treaded_base_core import (
        TreadedBase,
        TreadedBaseConfig,
        TreadedBaseController,
        TreadyEStopState,
        TreadyExitState,
    )

    family = unique_family("tready")
    group = imitation([(family, name) for name in TREADY_NAMES])[family]
    config = TreadedBaseConfig(
        hrdf_file=str(CONFIG_DIR / "hrdf" / "Tready.hrdf"),
        gains_file=str(CONFIG_DIR / "gains" / "smart-tready-gains.xml"),
        wheel_diameter=0.125,
        wheel_base=0.285,
        torso_torque_scale=2.5,
        torque_mode_max=25,
    )
    base = TreadedBase(config, group, chassis_ramp_time=0.5, flipper_ramp_time=0.1)
    # Imitated modules report the M-stop as pressed; drive it from the test instead.
    base.test_mstop = False
    monkeypatch.setattr(TreadedBase, "m_stopped", property(lambda self: self.test_mstop))
    ctrl = TreadedBaseController(base, TreadyInitialState, TreadyEStopState, TreadyExitState, max_base_speed=0.25)
    yield ctrl
    if base.running:
        base.stop()


def feed(ctrl, seconds, make_input):
    """Mobile IO stand-in: the base thread runs the controller; this sends it inputs (None = no Mobile IO sample)."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        inputs = make_input()
        if inputs is not None:
            ctrl.receive_inputs(inputs)
        time.sleep(0.02)


def wheel_velocity(ctrl):
    with ctrl.base.lock:
        return np.array(ctrl.base.wheel_cmd.velocity)


def flipper_cmd(ctrl):
    with ctrl.base.lock:
        return np.array(ctrl.base.flipper_cmd.position), np.array(ctrl.base.flipper_cmd.velocity)


def to_teleop(ctrl):
    from kits.tready.treaded_base import TreadyTeleopState
    from kits.tready.treaded_base_core import TreadyInputs

    feed(ctrl, 0.5, lambda: TreadyInputs(override_startup=True))
    assert isinstance(ctrl.state, TreadyTeleopState), ctrl.state


def test_disconnect_stops_the_base(base_control):
    """A9: with no Mobile IO input the last wheel velocity kept being sent, so the base kept driving."""
    from kits.tready.treaded_base_core import ChassisVelocity, TreadyDisconnectedState, TreadyInputs

    ctrl = base_control
    to_teleop(ctrl)
    feed(ctrl, 1.0, lambda: TreadyInputs(base_motion=ChassisVelocity(1.0, 0.0)))
    assert np.any(np.abs(wheel_velocity(ctrl)) > 0.1), "test precondition: base is driving"

    feed(ctrl, 1.5, lambda: None)
    assert isinstance(ctrl.state, TreadyDisconnectedState)
    assert np.allclose(wheel_velocity(ctrl), 0.0)
    assert np.allclose(flipper_cmd(ctrl)[1], 0.0)


def test_estop_holds_position_and_stops(base_control):
    """A9: E-stop kept a stale flipper position command; modules drive back to it when the M-stop is released."""
    from kits.tready.treaded_base_core import ChassisVelocity, TreadyEStopState, TreadyInputs

    ctrl, base = base_control, base_control.base
    to_teleop(ctrl)
    feed(ctrl, 0.5, lambda: TreadyInputs(base_motion=ChassisVelocity(1.0, 0.0)))

    base.test_mstop = True
    feed(ctrl, 0.5, lambda: TreadyInputs())
    assert isinstance(ctrl.state, TreadyEStopState)
    assert np.allclose(wheel_velocity(ctrl), 0.0)
    position, velocity = flipper_cmd(ctrl)
    assert np.allclose(velocity, 0.0)
    assert np.allclose(position, base.flipper_fbk.position, atol=0.05)  # holds where it is


def test_mstop_during_disconnect_recovers(base_control):
    """A10: ESTOP and DISCONNECTED handed control back and forth forever, never returning to the interrupted state."""
    from kits.tready.treaded_base import TreadyTeleopState
    from kits.tready.treaded_base_core import TreadyDisconnectedState, TreadyEStopState, TreadyInputs

    ctrl, base = base_control, base_control.base
    to_teleop(ctrl)
    feed(ctrl, 1.5, lambda: None)
    assert isinstance(ctrl.state, TreadyDisconnectedState)

    base.test_mstop = True
    feed(ctrl, 0.5, lambda: None)
    assert isinstance(ctrl.state, TreadyEStopState)

    base.test_mstop = False
    feed(ctrl, 0.5, lambda: TreadyInputs())
    assert isinstance(ctrl.state, TreadyTeleopState), ctrl.state


def test_estop_released_resumes_teleop(base_control):
    """Releasing the M-stop returns to the state the E-stop interrupted, without any Mobile IO reconnect."""
    from kits.tready.treaded_base import TreadyTeleopState
    from kits.tready.treaded_base_core import TreadyEStopState, TreadyInputs

    ctrl, base = base_control, base_control.base
    to_teleop(ctrl)
    base.test_mstop = True
    feed(ctrl, 0.5, lambda: TreadyInputs())
    assert isinstance(ctrl.state, TreadyEStopState)

    base.test_mstop = False
    feed(ctrl, 0.5, lambda: TreadyInputs())
    assert isinstance(ctrl.state, TreadyTeleopState), ctrl.state


def test_reconnect_does_not_restart_homing(base_control):
    """A10: reconnecting re-ran the previous state's entry action (here, a new homing move)."""
    from kits.tready.treaded_base import TreadyHomingState, TreadyTeleopState
    from kits.tready.treaded_base_core import TreadyDisconnectedState, TreadyInputs

    ctrl = base_control
    to_teleop(ctrl)
    ctrl.receive_inputs(TreadyInputs(home=True))
    assert isinstance(ctrl.state, TreadyHomingState)

    feed(ctrl, 1.5, lambda: None)
    assert isinstance(ctrl.state, TreadyDisconnectedState)
    feed(ctrl, 0.3, lambda: TreadyInputs())
    assert isinstance(ctrl.state, TreadyTeleopState), ctrl.state


def test_torque_toggle_handled_on_reset_sample(fake_mobile_io):
    """A12: update_drive_mode returned on the reset button before handling a torque-button toggle in the same sample."""
    from kits.tready.mobile_io_manager import update_drive_mode

    fake_mobile_io.press(b1=1, b2=1)
    update_drive_mode(fake_mobile_io)
    assert any(name == "set_snap" for name, _ in fake_mobile_io.calls)


def test_base_pose_with_nan_orientation():
    """A14: a NaN IMU quaternion (no orientation yet) made Rotation.from_quat raise inside the control loop."""
    import hebi

    from kits.tready.treaded_base_core import TreadedBase

    stub = SimpleNamespace(
        last_feedback=SimpleNamespace(position=np.zeros(8), orientation=np.full((8, 4), np.nan)),
        robot_model=hebi.robot_model.import_from_hrdf(str(CONFIG_DIR / "hrdf" / "Tready.hrdf")),
    )
    rpy = TreadedBase.pose.fget(stub)
    assert np.all(np.isfinite(rpy))
