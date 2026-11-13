"""Camera, pan/tilt mast and core sampler pieces that can be checked without hardware."""

from types import SimpleNamespace

import hebi
import numpy as np

from conftest import REPO_ROOT

NAN_QUAT = np.full(4, np.nan)


def test_mast_update_tolerates_nan_orientation(imitation):
    """A14: CameraMast raised on a NaN IMU quaternion (no orientation yet), killing the update thread."""
    from conftest import unique_family
    from kits.camera.pan_tilt_mast import CameraMast

    family = unique_family("mast")
    group = imitation([(family, "pan"), (family, "tilt")])[family]
    mast = CameraMast(group)

    class NoOrientationFeedback:  # what the modules report until the IMU has an orientation
        position = np.zeros(2)

        def copy_from(self, other):
            pass

        def __getitem__(self, i):
            return SimpleNamespace(orientation=NAN_QUAT)

    mast._feedback = NoOrientationFeedback()

    # What the update thread does with each new feedback: this must not raise.
    assert mast.update()
    assert np.all(np.isfinite(mast.output_frame[:3, 3]))


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
