from typing import List

import numpy as np

from .body import PeripheralBody, RobotModel
from util import math_utils


class Leg(PeripheralBody):
    """Represents a leg (and wheel)"""

    damper_gains = np.array([2.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    spring_gains = np.array([400.0, 0.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64)
    roll_gains = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0], dtype=np.float64)  # (N or Nm) / degree

    def __init__(self, val_lock, name, group_indices: List[int], robot_model: RobotModel):
        assert name == 'Left' or name == 'Right'
        super(Leg, self).__init__(val_lock, group_indices, robot_model)

        hip_t = np.identity(4, dtype=np.float64)
        hip_t[0:3, 0:3] = math_utils.rotate_x(np.pi * 0.5)
        hip_t[0:3, 3] = [0.0, 0.0225, 0.055]
        self._hip_t = hip_t

        home_knee_angle = np.deg2rad(130)
        home_hip_angle = (np.pi + home_knee_angle) * 0.5

        if name == 'Left':
            self._direction = 1.0
        else:
            self._direction = -1.0

        home_angles = np.array([home_hip_angle, home_knee_angle], dtype=np.float64)

        self.home_angles = self._direction * home_angles
        masses = self._kin.masses
        self._set_mass(np.sum(masses))
        self._set_masses(masses)

        # ----------------------
        # Populate cached fields
        output_frame_count = self._kin.get_frame_count('output')
        com_frame_count = self._kin.get_frame_count('CoM')

        self._current_xyz = np.zeros((3, com_frame_count), dtype=np.float64)

        self._current_fk = [np.zeros((4, 4), dtype=np.float64) for _ in range(output_frame_count)]
        self._current_coms = [np.zeros((4, 4), dtype=np.float64) for _ in range(com_frame_count)]

        # Calculate home FK
        self._hip_angle = home_hip_angle
        self._knee_angle = home_knee_angle
        self._knee_velocity = 0.0
        self._user_commanded_knee_velocity = 0.0
        self._knee_angle_max = 2.65
        self._knee_angle_min = 0.65
        self._e_term = np.empty(6, dtype=np.float64)

        # Additionally calculate commanded endeffector position
        self._current_cmd_tip_fk = np.zeros((4, 4), dtype=np.float64)

    def integrate_step(self, dt, knee_velocity):
        """Called by Igor. User should not call this directly.

        :param dt:            integral timestep
        :param knee_velocity: The calculated knee velocity
                              at the given instance in time
        """
        if (self._knee_angle > self._knee_angle_max) and (knee_velocity > 0.0) or\
           (self._knee_angle < self._knee_angle_min) and (knee_velocity < 0.0):
            # software controlled joint limit for the knee
            knee_velocity = 0.0

        self._knee_velocity = knee_velocity
        self._knee_angle = self._knee_angle + knee_velocity * dt
        self._hip_angle = (np.pi + self._knee_angle) * 0.5

    def update_position(self):
        """Updates calculations based on the position of the leg at the given
        point in time."""
        super(Leg, self).update_position()
        self._kin.get_forward_kinematics('endeffector', self._fbk.position_command, output=[self._current_cmd_tip_fk])

    def update_command(self, roll_angle, soft_start):
        """Write into the command object based on the current state of the leg.

        :param roll_angle:
        :param soft_start:
        :return:
        """

        # -------------------------------
        # Calculate position and velocity
        self._cmd.position = self._direction * np.array([self._hip_angle, self._knee_angle])
        self._cmd.velocity = self._direction * np.array([0.5 * self._knee_velocity, self._knee_velocity])

        # ----------------
        # Calculate effort

        # Calculate the current positional error
        np.subtract(self._current_cmd_tip_fk[0:3, 3], self.current_tip_fk[0:3, 3], out=self.xyz_error)
        # Calculate the current velocity error by:
        #  multiplying the current jacobian at the endeffector frame
        # by the:
        #  difference of the commanded velocity feedfback and actual velocity feedfback
        np.subtract(self._fbk.velocity_command, self._fbk.velocity, out=self._fbk_velocity_err)
        np.dot(self._current_j_actual_f, self._fbk_velocity_err, out=self._vel_error)

        # piecewise multiply the error terms by the predefined gains
        self._pos_error *= Leg.spring_gains
        self._vel_error *= Leg.damper_gains
        # `self._e_term` is an intermediate value used to calculate the impedance error
        np.multiply(Leg.roll_gains, self._direction * roll_angle, out=self._e_term)

        # add the 3 error terms (pos+vel+e) to find the impedance error
        np.add(self._pos_error, self._vel_error, out=self._impedance_err)
        np.add(self._impedance_err, self._e_term, out=self._impedance_err)
        # Multiply the jacobian matrix at the endeffector
        # by the impedance error to find the impedance torque, and scale it
        # by the soft startup scale
        np.dot(self._current_j_actual.T, self._impedance_err, out=self._impedance_torque)
        np.multiply(self._impedance_torque, soft_start, out=self._impedance_torque)

        self._cmd.effort = self._impedance_torque

    @property
    def hip_angle(self):
        """
        :return: The current hip position (in radians)
        :rtype:  float
        """
        return self._hip_angle

    @property
    def knee_angle(self):
        """
        :return: The current knee angle (in radians)
        :rtype:  float
        """
        return self._knee_angle

    @property
    def user_commanded_knee_velocity(self):
        """
        :return: (in rad/s)
        :rtype:  float
        """
        return self._user_commanded_knee_velocity

    def set_knee_velocity(self, vel):
        """
        :param vel: (in rad/s)
        :type vel:  float
        """
        self.acquire_value_lock()
        self._user_commanded_knee_velocity = vel
        self.release_value_lock()

    def reset_state(self):
        super(Leg, self).reset_state()

        self._knee_velocity = 0
        self._user_commanded_knee_velocity = 0
        self._e_term.fill(0)
        self._current_cmd_tip_fk.fill(0)
