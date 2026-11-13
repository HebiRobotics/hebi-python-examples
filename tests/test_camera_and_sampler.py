"""Camera, pan/tilt mast and core sampler pieces that can be checked without hardware."""

from types import SimpleNamespace

import hebi
import numpy as np

from conftest import REPO_ROOT

NAN_QUAT = np.full(4, np.nan)


def test_camera_tolerates_nan_orientation():
    """A14: HebiCamera computed Euler angles from a NaN quaternion, which raises (also in __init__)."""
    from kits.camera.camera import HebiCamera

    module = SimpleNamespace(gyro=np.zeros(3))

    class Feedback:  # group feedback: .orientation per module, indexable per module
        orientation = [NAN_QUAT]

        def __getitem__(self, i):
            return module

    stub = SimpleNamespace(fbk=Feedback(), roll=np.nan, group=SimpleNamespace(feedback_frequency=100.0))
    stub.ea_zxz = HebiCamera.ea_zxz.fget(stub)  # must not raise
    HebiCamera.update_image_rotation(stub)  # must not raise


def test_mast_update_tolerates_nan_orientation():
    """A14: HebiCameraMast.update raised on a NaN quaternion, killing the control loop."""
    from kits.camera.pan_tilt_mast import HebiCameraMast

    fbk_item = SimpleNamespace(orientation=NAN_QUAT)
    fbk = type("Fbk", (), {"position": np.zeros(2), "__getitem__": lambda self, i: fbk_item})()
    stub = SimpleNamespace(
        group=SimpleNamespace(get_next_feedback=lambda reuse_fbk=None: fbk),
        fbk=fbk,
        robot_model=hebi.robot_model.import_from_hrdf(str(REPO_ROOT / "kits" / "camera" / "hrdf" / "pan_tilt_mast.hrdf")),
        output_frame=np.empty((4, 4)),
        trajectory=None,
        cmd=SimpleNamespace(),
    )
    HebiCameraMast.update(stub, 0.0)  # must not raise


def test_mast_trajectory_starts_at_zero_acceleration():
    """A18: the mast trajectory used the commanded *effort* as its starting acceleration."""
    from kits.tready.treadward_sampler import CoreSampler

    stub = SimpleNamespace(
        mast_traj=None,
        mast_fbk=SimpleNamespace(position=np.array([0.1]), velocity=np.array([0.0]), effort_command=np.array([5.0])),
    )
    CoreSampler.set_mast_trajectory(stub, 0.0, 1.0, p=[0.5])
    _, _, acc = stub.mast_traj.get_state(0.0)
    assert np.allclose(acc, 0.0)
