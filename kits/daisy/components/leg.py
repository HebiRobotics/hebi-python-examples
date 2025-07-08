import numpy as np

from numpy.linalg import solve, LinAlgError
from hebi.robot_model import endeffector_position_objective
from util.math_utils import rotate_x, rotate_y, rotate_z, quat2rot

import hebi
from hebi.robot_model import FrameType

import typing
if typing.TYPE_CHECKING:
    import numpy.typing as npt
    from hebi._internal.ffi._message_types import GroupCommandView, GroupFeedbackView

class Leg:
    joint_count = 3

    __slots__ = ('_seed_angles', '_step', '_mode', '_index',
                 #
                 '_on_state_computed',
                 # Cartesian coordinates state
                 '_home_stance_xyz', '_level_home_stance_xyz',
                 '_cmd_stance_xyz', '_stance_vel_xyz',
                 # Leg parameters
                 '_stance_radius', '_body_height',
                 # Parameters for torque control
                 '_spring_shift', '_drag_shift',
                 # Static kinematics variables
                 '_kinematics',
                 # Workspace variables for kinematics
                 '_ee_frame', '_jacobian_ee', '_jacobian_coms',
                 # Views into HEBI objects
                 '_command_view', '_feedback_view')

    def __init__(self, index: int, leg_kinematics: 'hebi.robot_model.RobotModel', parameters, command_view: 'GroupCommandView', feedback_view: 'GroupFeedbackView'):
        self._index = index

        num_joints = leg_kinematics.dof_count
        seed_angles = np.array([0.2, 0.3, 1.9], dtype=np.float64)
        spring_shift = 3.75

        # Leg config specific values
        if index % 2 == 0:
            spring_shift *= -1.0
            seed_angles[1:] *= -1

        self._spring_shift = spring_shift
        self._drag_shift = 1.5

        self._stance_radius = parameters.stance_radius
        self._body_height = parameters.default_body_height

        frames = leg_kinematics.get_forward_kinematics(FrameType.Output, seed_angles)
        base_frame = frames[0]

        # Calculate stance in cartesian coordinates

        self._seed_angles = seed_angles
        self._home_stance_xyz = (base_frame[0:3, 0:3] @ [self._stance_radius, 0.0, -self._body_height])[0:3]
        self._level_home_stance_xyz = self.home_stance_xyz.copy()
        self._ee_frame = leg_kinematics.get_end_effector(feedback_view.position)
        self._cmd_stance_xyz = leg_kinematics.get_end_effector(seed_angles)[0:3, 3]
        self._stance_vel_xyz = np.zeros(3, dtype=np.float64)
        self._kinematics = leg_kinematics

        from .step import Step
        self._step = Step(self.cmd_stance_xyz.copy())

        com_frame_count = self.kinematics.get_frame_count(FrameType.CenterOfMass)

        self._jacobian_ee = np.zeros((6, num_joints), dtype=np.float64)
        self._jacobian_coms = np.empty((6, num_joints, com_frame_count), np.float64)

        self._command_view = command_view
        self._feedback_view = feedback_view

        self._on_state_computed = lambda p, v, e, leg: None

    def track_trajectory(self, t, trajectory, gravity, foot_force):
        """Follows the provided trajectory, while additionally compensating for
        gravity and ground forces."""
        angles, vel, a = trajectory.get_state(t)
        self._command_view.position[:] = angles
        self._command_view.velocity[:] = vel

        # Compute jacobians
        self.kinematics.get_jacobian_end_effector(angles, output=self._jacobian_ee)
        self.kinematics.get_jacobians_mat(FrameType.CenterOfMass, angles, output=self._jacobian_coms)

        self.compute_torques(gravity, foot_force, a)

    def get_state_at_time(self, t):
        """Used to determine calculated positions and velocities without
        sending them to the modules."""
        velocities = None

        if self._step.active:
            positions, velocities, _ = self._step.compute_state(t)
        else:
            positions = self.kinematics.solve_inverse_kinematics(self._seed_angles, endeffector_position_objective(self._cmd_stance_xyz))
            # For stance mode, also compute velocities
            jacobian_ee = self.kinematics.get_jacobian_end_effector(positions)

            try:
                # Note: C++ uses column pivoted householder QR to solve here. Not sure if the additional precision is necessary
                velocities = np.array(solve(jacobian_ee[0:3, :], self.stance_vel_xyz), dtype=np.float32)
            except LinAlgError as lin:
                velocities = np.zeros(self.kinematics.dof_count, dtype=np.float32)

        return positions, velocities

    def compute_torques(self, gravity, foot_force, joint_accelerations):
        """
        TODO: Document
        """
        view = self._command_view
        self.kinematics.get_grav_comp_efforts(self._feedback_view.position, gravity, self._jacobian_coms, output=view.effort.astype(np.float64))
        view.effort += self._jacobian_ee[0:3, :].T @ -foot_force
        view.effort[1] += self._spring_shift + self._drag_shift * view.velocity[1]
        if self._step._trajectory is not None:
            self._step._trajectory.get_state
            view.effort += self.kinematics.get_dynamic_comp_efforts(view.position, view.position, view.velocity.astype(np.float64), joint_accelerations)

    def compute_state(self, t, gravity, foot_force):
        """
        TODO: Document
        """
        view = self._command_view

        if self._step.active:
            self.track_trajectory(t, self._step._trajectory, gravity, foot_force)
            self._on_state_computed(view.position, view.velocity, view.effort, self)
            return

        self.kinematics.solve_inverse_kinematics(self._seed_angles, endeffector_position_objective(self.cmd_stance_xyz), output=view.position)
        self.kinematics.get_jacobians_mat('com', view.position, output=self._jacobian_coms)

        # Compute jacobians
        jacobian_ee = self._jacobian_ee
        self.kinematics.get_jacobian_end_effector(view.position, output=jacobian_ee)

        # For stance mode, also compute velocities
        try:
            # Note: C++ uses column pivoted householder QR to solve here. Not sure if the additional precision is necessary
            view.velocity[:] = solve(jacobian_ee[0:3, :], self.stance_vel_xyz)
        except LinAlgError as lin:
            view.velocity.fill(0.0)

        self.compute_torques(gravity, foot_force, np.zeros(self.joint_count))

        self._on_state_computed(view.position, view.velocity, view.effort, self)

    def update_stance(self, translation_vel: 'npt.NDArray[np.float64]', rotate_vel: 'npt.NDArray[np.float64]', dt):
        """
        TODO: Document
        """

        # stance_vel_xyz = translation_vel + rotate_vel.cross(cmd_stance_xyz)
        self._stance_vel_xyz[:] = translation_vel + np.cross(rotate_vel, self._cmd_stance_xyz)

        # FIXME: Cut down on allocations here
        self._cmd_stance_xyz += translation_vel * dt
        self._cmd_stance_xyz[:] = rotate_z(rotate_vel[2] * dt) @ rotate_y(rotate_vel[1] * dt) @ rotate_x(rotate_vel[0] * dt) @ self._cmd_stance_xyz

        self.kinematics.get_end_effector(self._feedback_view.position, output=self._ee_frame)

        # Update home stance to match the current z height
        self._level_home_stance_xyz[2] += translation_vel[2] * dt
        self._home_stance_xyz[:] = rotate_x(0.2 * translation_vel[1]) @ rotate_y(-0.2 * translation_vel[0]) @ self._level_home_stance_xyz

    def start_step(self, t):
        self._step.reset(t, self)

    def update_step(self, t):
        if not self._step.active:
            return
        if self._step.update(t, self):
            # Transition to stance mode
            np.copyto(self._cmd_stance_xyz, self._step.touch_down)

    def get_local_gravity(self):
        orientation = self._feedback_view.orientation[0]
        rot_matrix = quat2rot(orientation)
        return self.kinematics.base_frame[0:3, 0:3] @ rot_matrix.T @ np.array([0, 0, -1], dtype=np.float64)

    def get_foot_contact_factor(self, t):
        if self._step._trajectory is None:
            return 1.0

        switch_time = 0.1
        current_step_time = t - self._step._trajectory.start_time
        step_period = self.step_period

        if current_step_time < switch_time:
            return (switch_time - current_step_time) / switch_time
        elif (step_period - current_step_time) < switch_time:
            return (current_step_time - (step_period - switch_time)) / switch_time
        return 0.0

    def get_stance_radius(self, t, gravity_dir):
        stance = self.cmd_stance_xyz
        dot_product = -gravity_dir.dot(stance)
        foot_stance_radius = np.linalg.norm(-gravity_dir * dot_product - stance)
        return foot_stance_radius

    @property
    def level_home_stance_xyz(self):
        return self._level_home_stance_xyz

    @property
    def home_stance_xyz(self):
        return self._home_stance_xyz

    @property
    def cmd_stance_xyz(self):
        return self._cmd_stance_xyz

    @property
    def fbk_stance_xyz(self):
        return self._ee_frame[0:3, 3]

    @property
    def stance_vel_xyz(self):
        return self._stance_vel_xyz

    @property
    def seed_angles(self):
        return self._seed_angles

    @property
    def kinematics(self):
        return self._kinematics

    @property
    def mode(self):
        return 'flight' if self._step.active else 'stance'

    @property
    def step_period(self):
        return self._step.period
