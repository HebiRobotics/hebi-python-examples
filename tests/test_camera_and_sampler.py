"""Camera mast and core sampler, against imitated modules where they need hardware."""

import time
from types import SimpleNamespace

import hebi
import numpy as np
import pytest

from conftest import unique_family


@pytest.fixture
def mast_control(imitation, monkeypatch):
    """A started MastController on an imitated mast + camera."""
    from kits.camera import pan_tilt_mast as ptm
    from kits.camera.camera import HebiCamera

    mast_family, camera_family = unique_family("mast"), unique_family("cam")
    groups = imitation([(mast_family, "J1_pan"), (mast_family, "J2_tilt"), (camera_family, "toolhead"), (camera_family, "cam1")])
    # Imitated modules report the M-stop as pressed; drive it from the test instead.
    monkeypatch.setattr(ptm.CameraMast, "m_stopped", property(lambda self: getattr(self, "test_mstop", False)))
    mast = ptm.CameraMast(groups[mast_family])
    camera = HebiCamera(groups[camera_family])
    ctrl = ptm.MastController(mast, camera, ptm.CameraMastStartupState, ptm.CameraMastFaultState, ptm.CameraMastExitState)
    yield ctrl
    for manager in (mast, camera):
        if manager.running:
            manager.stop()


def feed(ctrl, seconds, **inputs):
    from kits.camera.pan_tilt_mast import MastInputs

    values = dict(v_pan=0.0, v_tilt=0.0, home=False, zoom=1.0, light_enable=False, light_pwm=0.0) | inputs
    deadline = time.time() + seconds
    while time.time() < deadline:
        ctrl.receive_inputs(MastInputs(**values))
        time.sleep(0.02)


def wait_for(ctrl, cls, timeout=5.0, **inputs):
    deadline = time.time() + timeout
    while time.time() < deadline and not isinstance(ctrl.state, cls):
        feed(ctrl, 0.05, **inputs)
    return isinstance(ctrl.state, cls)


def test_mast_holds_on_disconnect(mast_control):
    """The disconnected state cancelled the goal, leaving the mast limp (the tilt could sag)."""
    from kits.camera import pan_tilt_mast as ptm

    ctrl = mast_control
    assert wait_for(ctrl, ptm.CameraMastTeleopState, timeout=8)
    feed(ctrl, 0.3, v_pan=1.0)
    time.sleep(1.5)  # no inputs
    assert isinstance(ctrl.state, ptm.CameraMastDisconnectedState)
    traj = ctrl.mast.trajectory
    assert traj is not None, "mast has no goal (limp)"
    assert np.allclose(traj.get_state(traj.end_time)[1], 0.0)


def test_mast_mstop_drops_goal_and_recovers(mast_control):
    """The demos passed the disconnected state as the M-stop state; with disconnect holding, an M-stop
    would hold a target the modules drive back to on release."""
    from kits.camera import pan_tilt_mast as ptm

    ctrl = mast_control
    assert wait_for(ctrl, ptm.CameraMastTeleopState, timeout=8)
    ctrl.mast.test_mstop = True
    assert wait_for(ctrl, ptm.CameraMastFaultState, timeout=2)
    assert ctrl.mast.trajectory is None
    ctrl.mast.test_mstop = False
    assert wait_for(ctrl, ptm.CameraMastTeleopState, timeout=2)


def test_mast_survives_nan_orientation(imitation, monkeypatch):
    """CameraMast._inner_update raised on the NaN quaternion reported before the IMU has an orientation,
    killing the mast's control thread."""
    from kits.camera import pan_tilt_mast as ptm

    family = unique_family("mast")
    group = imitation([(family, "J1_pan"), (family, "J2_tilt")])[family]
    mast = ptm.CameraMast(group)
    nan_feedback = type("Fbk", (), {"position": np.zeros(2), "__getitem__": lambda self, i: SimpleNamespace(orientation=np.full(4, np.nan))})()
    monkeypatch.setattr(ptm.CameraMast, "last_feedback", property(lambda self: nan_feedback))
    assert mast.update()  # must not raise


def test_mast_trajectory_starts_at_zero_acceleration():
    """The mast trajectory used the commanded *effort* as its starting acceleration."""
    from kits.tready.treadward_sampler import CoreSampler

    stub = SimpleNamespace(
        mast_traj=None,
        mast_fbk=SimpleNamespace(position=np.array([0.1]), velocity=np.array([0.0]), effort_command=np.array([5.0])),
    )
    CoreSampler.set_mast_trajectory(stub, 0.0, 1.0, p=[0.5])
    _, _, acc = stub.mast_traj.get_state(0.0)
    assert np.allclose(acc, 0.0)
