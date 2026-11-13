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
    from kits.tready.treaded_base_core import TreadedBase, TreadedBaseConfig, TreadedBaseControl

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
    monkeypatch.setattr(TreadedBase, "mstop_pressed", property(lambda self: self.test_mstop))
    return TreadedBaseControl(base, max_base_speed=0.25)


def run(ctrl, seconds, make_input):
    """Main-loop stand-in, as in kits/tready/tready.py. make_input() -> TreadyInputs or None (no Mobile IO sample)."""
    from kits.tready.tready import update_inputs

    deadline = time.time() + seconds
    while time.time() < deadline:
        inputs = make_input()
        ctrl.update(time.time(), None if inputs is None else update_inputs(inputs))
        ctrl.send()


def to_teleop(ctrl):
    from kits.tready.treaded_base_core import TreadyControlState, TreadyInputs

    run(ctrl, 0.5, lambda: TreadyInputs(override_startup=True))
    assert ctrl.state is TreadyControlState.TELEOP, ctrl.state


def test_disconnect_stops_the_base(base_control):
    """A9: with no Mobile IO input, update() returned before base.update(), so the last wheel velocity kept being sent."""
    from kits.tready.treaded_base_core import ChassisVelocity, TreadyControlState, TreadyInputs

    ctrl, base = base_control, base_control.base
    to_teleop(ctrl)
    run(ctrl, 1.0, lambda: TreadyInputs(base_motion=ChassisVelocity(1.0, 0.0)))
    assert np.any(np.abs(base.wheel_cmd.velocity) > 0.1), "test precondition: base is driving"

    run(ctrl, 1.5, lambda: None)
    assert ctrl.state is TreadyControlState.DISCONNECTED
    assert np.allclose(base.wheel_cmd.velocity, 0.0)
    assert np.allclose(base.flipper_cmd.velocity, 0.0)


def test_estop_leaves_no_position_target(base_control):
    """A9: E-stop kept the last flipper position command; modules drive back to it when the M-stop is released."""
    from kits.tready.treaded_base_core import ChassisVelocity, TreadyControlState, TreadyInputs

    ctrl, base = base_control, base_control.base
    to_teleop(ctrl)
    run(ctrl, 0.5, lambda: TreadyInputs(base_motion=ChassisVelocity(1.0, 0.0)))

    base.test_mstop = True
    run(ctrl, 0.2, lambda: TreadyInputs())
    assert ctrl.state is TreadyControlState.EMERGENCY_STOP
    assert np.all(np.isnan(base.flipper_cmd.position))
    assert np.allclose(base.wheel_cmd.velocity, 0.0)


def test_mstop_during_disconnect_recovers(base_control):
    """A10: prev_state could become an error state, so ESTOP and DISCONNECTED handed control back and forth forever."""
    from kits.tready.treaded_base_core import TreadyControlState, TreadyInputs

    ctrl, base = base_control, base_control.base
    to_teleop(ctrl)
    run(ctrl, 1.5, lambda: None)
    assert ctrl.state is TreadyControlState.DISCONNECTED

    base.test_mstop = True
    run(ctrl, 0.2, lambda: None)
    assert ctrl.state is TreadyControlState.EMERGENCY_STOP

    base.test_mstop = False
    run(ctrl, 0.5, lambda: TreadyInputs())
    assert ctrl.state is TreadyControlState.TELEOP


def test_reconnect_does_not_restart_homing(base_control):
    """A10: reconnecting re-ran the previous state's entry action (here, a new homing move)."""
    from kits.tready.treaded_base_core import TreadyControlState, TreadyInputs

    ctrl = base_control
    to_teleop(ctrl)
    ctrl.update(time.time(), None)
    from kits.tready.tready import update_inputs

    ctrl.update(time.time(), update_inputs(TreadyInputs(home=True)))
    assert ctrl.state is TreadyControlState.HOMING

    run(ctrl, 1.5, lambda: None)
    assert ctrl.state is TreadyControlState.DISCONNECTED
    run(ctrl, 0.1, lambda: TreadyInputs())
    assert ctrl.state is TreadyControlState.TELEOP


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
        fbk=SimpleNamespace(position=np.zeros(8), orientation=np.full((8, 4), np.nan)),
        robot_model=hebi.robot_model.import_from_hrdf(str(CONFIG_DIR / "hrdf" / "Tready.hrdf")),
    )
    rpy = TreadedBase.pose.fget(stub)
    assert np.all(np.isfinite(rpy))
