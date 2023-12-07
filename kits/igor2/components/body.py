from typing import List, Optional
import numpy as np

import hebi
from hebi.robot_model import RobotModel
from hebi._internal.ffi._message_types import GroupCommand, GroupFeedback

from util.type_utils import assert_type
from util import math_utils


class BaseBody(object):
    """Base class for all body components of Igor."""

    def __init__(self, val_lock, mass=0.0, com=[0.0, 0.0, 0.0]):
        self.__val_lock = val_lock
        self._mass = mass
        self._com = np.zeros(3, dtype=np.float64)
        self._com[0] = com[0]
        self._com[1] = com[1]
        self._com[2] = com[2]

    def _set_mass(self, mass):
        """Called by subclasses to update mass."""
        self._mass = mass

    def _set_com(self, com):
        """Called by subclasses to update the center of mass."""
        self._com[0:3] = com

    def acquire_value_lock(self):
        """Used to acquire mutex for the parameters of this body component."""
        self.__val_lock.acquire()

    def release_value_lock(self):
        """Used to release mutex for the parameters of this body component."""
        self.__val_lock.release()

    @property
    def mass(self):
        """
        :return: The mass (in kilograms) of this body component
        :rtype:  float
        """
        return self._mass

    @property
    def com(self):
        """
        :return: The center of mass (in meters) of this body component in 3D Euclidean space
        :rtype:  np.ndarray
        """
        return self._com


class PeripheralBody(BaseBody):
    """Base class for all peripheral body components of Igor (e.g., legs and
    arms)"""

    def __init__(self, val_lock, group_indices: List[int], robot_model: RobotModel, mass=0.0, com=[0.0, 0.0, 0.0]):
        super(PeripheralBody, self).__init__(val_lock, mass, com)

        self._kin = robot_model
        self.group_indices = group_indices

        num_modules = len(group_indices)
        # ----------------------------------
        # Everything here is a column vector
        self._fbk_velocity_err = np.empty(num_modules, dtype=np.float32)  # doesn't need init b/c only used as placeholder
        self._pos_error = np.zeros(6, dtype=np.float64)
        self._vel_error = np.zeros(6, dtype=np.float32)
        self._impedance_err = np.zeros(6, dtype=np.float64)
        self._impedance_torque = np.zeros(num_modules, dtype=np.float64)
        self._home_angles = np.zeros(num_modules, dtype=np.float64)
        self._masses = None

        # Subclass populates these two empty list fields
        self._current_coms = list()
        self._current_fk = list()
        self._current_j_actual = np.zeros((6, num_modules), dtype=np.float64)
        self._current_j_actual_f = np.zeros((6, num_modules), dtype=np.float32)
        self._current_j_expected = np.zeros((6, num_modules), dtype=np.float64)
        # Subclass populates this as size of (3, numOfCoMFrames)
        self._current_xyz: Optional[np.ndarray] = None

    def set_message_views(self, cmd: GroupCommand, fbk: GroupFeedback):
        self._cmd = cmd.create_view(self.group_indices)
        self._fbk = fbk.create_view(self.group_indices)

    def _set_masses(self, masses):
        length = len(masses)
        dst_masses = np.empty(length, dtype=np.float64)
        np.copyto(dst_masses, masses)
        self._masses = dst_masses

    @property
    def home_angles(self):
        """
        :return: The home angle (in radians) positions of this body component
        :rtype:  np.array
        """
        return self._home_angles

    @home_angles.setter
    def home_angles(self, value):
        np.copyto(self._home_angles, value)

    @property
    def xyz_error(self):
        return self._pos_error[0:3]

    @property
    def current_coms(self):
        """
        :return:
        :rtype:
        """
        return self._current_coms

    @property
    def current_tip_fk(self):
        """
        :return:
        :rtype:
        """
        return self._current_fk[-1]

    @property
    def current_j_expected(self):
        """
        :return:
        :rtype:
        """
        return self._current_j_expected

    def get_grav_comp_efforts(self, gravity):
        """
        :param positions:
        :type positions:  np.array
        :param gravity:
        :type gravity:    np.array

        :return:
        :rtype:  np.array
        """
        return self._kin.get_grav_comp_efforts(self._fbk.position, gravity)

    def create_home_trajectory(self, duration=3.0):
        """Create a trajectory from the current pose to the home pose. This is
        used on soft startup.

        :param positions:
        :type positions:  np.array
        :param duration:  The total time of the calculated trajectory (in seconds)
        :type duration:   float

        :rtype: hebi._internal.trajectory.Trajectory

        :raises ValueError: If ``duration`` is less than 1.0
        :raises TypeError:  If ``duration`` is not of type float
        """
        assert_type(duration, float, 'duration')
        if duration < 1.0:
            raise ValueError('duration must be greater than 1.0 second')
        num_joints = self._cmd.size
        num_waypoints = 2
        dim = (num_joints, num_waypoints)

        home_angles = self._home_angles

        times = np.array([0.0, duration], dtype=np.float64)
        pos = np.empty(dim, dtype=np.float64)
        vel = np.zeros(dim, dtype=np.float64)
        accel = vel

        pos[:, 0] = self._fbk.position
        pos[:, 1] = home_angles

        return hebi.trajectory.create_trajectory(times, pos, vel, accel)

    def update_position(self):
        """Update kinematics from feedback."""
        positions = self._fbk.position

        robot = self._kin
        robot.get_forward_kinematics('com', positions, output=self._current_coms)
        robot.get_forward_kinematics('output', positions, output=self._current_fk)
        robot.get_jacobian_end_effector(positions, output=self._current_j_actual)
        robot.get_jacobian_end_effector(self._fbk.position_command, output=self._current_j_expected)

        np.copyto(self._current_j_actual_f, self._current_j_actual)

        for i, entry in enumerate(self._current_coms):
            self._current_xyz[0:3, i] = entry[0:3, 3]

        masses = self._masses
        np.sum(np.multiply(self._current_xyz, matlib.repmat(masses.T, 3, 1)), axis=1, out=self._com)
        self._com *= 1.0 / self.mass

    def reset_state(self):
        """Used when transitioning Igor back into idle mode."""
        self._pos_error.fill(0)
        self._vel_error.fill(0)
        self._impedance_err.fill(0)
        self._impedance_torque.fill(0)
        self._current_j_actual.fill(0)
        self._current_j_actual_f.fill(0)
        self._current_j_expected.fill(0)

        if self._current_xyz is not None:
            self._current_xyz.fill(0)
