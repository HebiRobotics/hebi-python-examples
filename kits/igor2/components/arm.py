from typing import List

import numpy as np
import hebi

from .body import PeripheralBody, RobotModel


class Arm(PeripheralBody):

    damper_gains = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64).T
    spring_gains = np.array([100.0, 10.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64).T

    Jacobian_Determinant_Threshold = 0.020
    """
  The lower threshold allowed for the determinant calculation of the jacobians.
  Anything below this will be considered at or near a singularity.
  """

    def __init__(self, val_lock, name, group_indices: List[int], robot_model: RobotModel):
        super(Arm, self).__init__(val_lock, group_indices, robot_model)

        self._user_commanded_grip_velocity = np.zeros(3, dtype=np.float64)

        if name == 'Left':
            self._direction = 1.0
        else:
            self._direction = -1.0

        home_angles = self._direction * np.array([0.0, 20.0, 60.0, 0.0], dtype=np.float64)
        home_angles = np.deg2rad(home_angles)

        # ----------------------
        # Populate cached fields
        output_frame_count = self._kin.get_frame_count('output')
        com_frame_count = self._kin.get_frame_count('CoM')

        self._current_xyz = np.zeros((3, com_frame_count), dtype=np.float64)

        for i in range(output_frame_count):
            self._current_fk.append(np.zeros((4, 4), dtype=np.float64))

        for i in range(com_frame_count):
            self._current_coms.append(np.zeros((4, 4), dtype=np.float64))

        # Calculate home FK
        self.home_angles = home_angles
        self._home_fk = self._kin.get_forward_kinematics('output', home_angles)
        self._home_ef = self._home_fk[-1]
        self._set_mass(np.sum(self._kin.masses))
        self._set_masses(self._kin.masses)

        self._grip_pos = np.array(self._home_ef[0:3, 3])
        self._new_grip_pos = self._grip_pos.copy()
        self._joint_angles = home_angles.copy()
        self._joint_velocities = np.zeros(4, np.float32)
        self._joint_efforts = np.zeros(4, np.float32)
        self._user_commanded_wrist_velocity = 0.0

        self._grav_comp_torque = np.zeros(len(home_angles), np.float64)

        # Additionally, calculate determinant of jacobians
        self._current_det_actual = 0.0
        self._current_det_expected = 0.0

    def integrate_step(self, dt, calculated_grip_velocity):
        """Called by Igor. User should not call this directly.

        :param dt:
        :type dt:                        float
        :param positions:
        :type positions:                 np.array
        :param calculated_grip_velocity:
        :type calculated_grip_velocity:  np.array
        """

        # Make endeffector velocities mirrored in Y
        adjusted_grip_v_term = calculated_grip_velocity.reshape((3,)).copy()
        adjusted_grip_v_term[1] *= self._direction

        # Integrate the adjusted grip velocity term to find the new grip position
        np.multiply(adjusted_grip_v_term, dt, out=adjusted_grip_v_term)
        np.add(self._grip_pos, adjusted_grip_v_term, out=self._new_grip_pos)

        xyz_objective = hebi.robot_model.endeffector_position_objective(self._new_grip_pos)
        new_arm_joint_angs = self._kin.solve_inverse_kinematics(self._fbk.position, xyz_objective)

        # Find the determinant of the jacobian at the endeffector of the solution
        # to the IK. If below a set threshold, set the joint velocities to zero
        # in an attempt to avoid nearing the kinematic singularity.
        jacobian_new = self._kin.get_jacobian_end_effector(new_arm_joint_angs)[0:3, 0:3]
        det_J_new = abs(np.linalg.det(jacobian_new))

        if (self._current_det_expected < Arm.Jacobian_Determinant_Threshold) and (det_J_new < self._current_det_expected):
            # Near singularity - don't command arm towards it
            self._joint_velocities[0:3] = 0.0
        else:
            try:
                self._joint_velocities[0:3] = np.linalg.solve(self._current_j_actual_f[0:3, 0:3], self._user_commanded_grip_velocity).reshape((3,))
                self._joint_angles[0:3] = new_arm_joint_angs[0:3]
                np.copyto(self._grip_pos, self._new_grip_pos)
            except np.linalg.LinAlgError as lin:
                # This may happen still sometimes
                self._joint_velocities[0:3] = 0.0

        wrist_vel = self._direction * self._user_commanded_wrist_velocity
        self._joint_velocities[3] = self._joint_velocities[1] + self._joint_velocities[2] + wrist_vel
        self._joint_angles[3] = self._joint_angles[3] + (self._joint_velocities[3] * dt)

    @property
    def current_det_actual(self):
        """
        :return:
        :rtype:  float
        """
        return self._current_det_actual

    @property
    def current_det_expected(self):
        """
        :return:
        :rtype:  float
        """
        return self._current_det_expected

    @property
    def user_commanded_grip_velocity(self):
        """
        :return:
        :rtype:  float
        """
        return self._user_commanded_grip_velocity

    @property
    def user_commanded_wrist_velocity(self):
        """
        :return:
        :rtype:  float
        """
        return self._user_commanded_wrist_velocity

    @property
    def grip_position(self):
        """
        :return:
        :rtype:  np.array
        """
        return self._grip_pos

    def update_position(self):
        """Updates calculations based on the position of the arm at the given
        point in time."""
        super(Arm, self).update_position()
        self._current_det_actual = abs(np.linalg.det(self._current_j_actual[0:3, 0:3]))
        self._current_det_expected = abs(np.linalg.det(self._current_j_expected[0:3, 0:3]))

    def update_command(self, pose, soft_start):
        """Write into the command object based on the current state of the arm.

        :param group_command:
        :param pose:
        :param soft_start:
        """

        # ----------------
        # Calculate effort
        np.subtract(self._grip_pos, self.current_tip_fk[0:3, 3], out=self.xyz_error)
        np.subtract(self._joint_velocities, self._fbk.velocity, out=self._vel_error[0:4])
        np.dot(self._current_j_actual_f, np.asarray(self._vel_error[0:4]), out=self._vel_error)

        self._pos_error *= Arm.spring_gains
        self._vel_error *= Arm.damper_gains
        np.add(self._pos_error, self._vel_error, out=self._impedance_err)
        np.dot(self._current_j_actual.T, np.asarray(self._impedance_err), out=self._impedance_torque)
        np.copyto(self._grav_comp_torque.ravel(), self._kin.get_grav_comp_efforts(self._fbk.position, -pose[2, 0:3]))

        np.multiply(soft_start, self._impedance_torque, out=self._joint_efforts)
        np.add(self._joint_efforts, self._grav_comp_torque, out=self._joint_efforts)

        # Set command
        self._cmd.position = self._joint_angles
        self._cmd.velocity = self._joint_velocities
        self._cmd.effort = self._joint_efforts

    def set_x_velocity(self, value):
        """
        :param value: the x velocity of the arm
        :type value:  float
        """
        self.acquire_value_lock()
        self._user_commanded_grip_velocity[0] = value
        self.release_value_lock()

    def set_y_velocity(self, value):
        """
        :param value: the y velocity of the arm
        :type value:  float
        """
        self.acquire_value_lock()
        self._user_commanded_grip_velocity[1] = value
        self.release_value_lock()

    def set_z_velocity(self, value):
        """
        :param value: the z velocity of the arm
        :type value:  float
        """
        self.acquire_value_lock()
        self._user_commanded_grip_velocity[2] = value
        self.release_value_lock()

    def set_wrist_velocity(self, value):
        """
        :param value: the velocity of the wrist (in rad/s)
        :type value:  float
        """
        self.acquire_value_lock()
        self._user_commanded_wrist_velocity = value
        self.release_value_lock()

    def reset_state(self):
        super(Arm, self).reset_state()

        self._user_commanded_grip_velocity.fill(0)
        self._current_xyz.fill(0)
        np.copyto(self._grip_pos, self._home_ef[0:3, 3])
        np.copyto(self._new_grip_pos, self._grip_pos)
        np.copyto(self._joint_angles, self.home_angles)
        self._joint_velocities.fill(0)
        self._joint_efforts.fill(0)
        self._user_commanded_wrist_velocity = 0.0
        self._grav_comp_torque.fill(0)
        self._current_det_actual = 0.0
        self._current_det_expected = 0.0
