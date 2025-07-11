import numpy as np

from numpy import float32, float64
from numpy.linalg import solve, LinAlgError
from hebi.robot_model import endeffector_position_objective
from util.math_utils import rotate_x, rotate_y, rotate_z


class Leg:
    joint_count = 3

    __slots__ = ('_seed_angles', '_step', '_mode', '_index',
                 #
                 '_on_state_computed',
                 # Cartesian coordinates state
                 '_home_stance_xyz', '_level_home_stance_xyz', '_fbk_stance_xyz',
                 '_cmd_stance_xyz', '_stance_vel_xyz',
                 # Leg parameters
                 '_stance_radius', '_body_height',
                 # Parameters for torque control
                 '_spring_torque', '_spring_shift', '_drag_shift',
                 # Static kinematics variables
                 '_kinematics', '_masses',
                 # Workspace variables for kinematics
                 '_ee_frame', '_jacobian_ee', '_jacobian_coms',
                 # Pending command fields
                 '_commanded_positions', '_commanded_velocities', '_commanded_efforts',
                 # Views into HEBI objects
                 '_command_view', '_feedback_view')

    def __init__(self, index, angle, distance, parameters, leg_configuration, command_view, feedback_view):
        self._index = index

        num_joints = Leg.joint_count
        from os.path import join
        from hebi.robot_model import import_from_hrdf
        kin = import_from_hrdf(join(parameters.resource_directory, '{0}.hrdf'.format(leg_configuration)))
        seed_angles = np.empty(num_joints, dtype=float64)

        # Leg config specific values
        if leg_configuration == 'left':
            spring_shift = -3.75
            seed_angles[:] = [0.2, -0.3, -1.9]
        else:
            spring_shift = 3.75
            seed_angles[:] = [0.2, 0.3, 1.9]

        self._spring_shift = spring_shift
        self._drag_shift = 1.5

        self._stance_radius = parameters.stance_radius
        self._body_height = parameters.default_body_height

        # Set base frame for kinematic body
        base_frame = np.identity(4, dtype=float64)
        base_frame[0:3, 0:3] = rotate_z(angle)
        base_frame[0:3, 3] = base_frame[0:3, 0:3] @ [distance, 0.0, 0.0]
        kin.base_frame = base_frame

        # Calculate stance in cartesian coordinates
        home_stance_xyz = np.empty(3, dtype=float64)
        level_home_stance_xyz = np.empty(3, dtype=float64)
        fbk_stance_xyz = np.empty(3, dtype=float64)
        cmd_stance_xyz = np.empty(3, dtype=float64)

        home_stance_xyz[:] = (base_frame[0:3, 0:3] @ [self._stance_radius, 0.0, -self._body_height])[0:3]
        np.copyto(level_home_stance_xyz, home_stance_xyz)

        fbk_stance_xyz[:] = kin.get_end_effector(feedback_view.position)[0:3, 3]
        cmd_stance_xyz[:] = kin.get_end_effector(seed_angles)[0:3, 3]

        from .step import Step

        self._seed_angles = seed_angles
        self._step = Step(0.0, cmd_stance_xyz.copy())
        self._mode = 'stance'
        self._home_stance_xyz = home_stance_xyz
        self._level_home_stance_xyz = level_home_stance_xyz
        self._fbk_stance_xyz = fbk_stance_xyz
        self._cmd_stance_xyz = cmd_stance_xyz
        self._stance_vel_xyz = np.zeros(3, dtype=float64)
        self._kinematics = kin
        self._masses = kin.masses
        self._spring_torque = np.zeros(num_joints, dtype=float64)

        self._commanded_positions = np.zeros(num_joints, dtype=float64)
        self._commanded_velocities = np.zeros(num_joints, dtype=float32)
        self._commanded_efforts = np.zeros(num_joints, dtype=float32)

        jacobian_ee = np.zeros((6, num_joints), dtype=float64)
        com_frame_count = self._kinematics.get_frame_count('com')
        jacobian_coms = [None] * com_frame_count
        for i in range(com_frame_count):
            jacobian_coms[i] = np.zeros((6, num_joints), dtype=float64)

        self._jacobian_coms = jacobian_coms
        self._jacobian_ee = jacobian_ee
        self._ee_frame = np.identity(4, dtype=float64)

        self._command_view = command_view
        self._feedback_view = feedback_view

        self._on_state_computed = lambda p, v, e, leg: None

    def track_trajectory(self, t, trajectory, gravity, foot_force):
        """Follows the provided trajectory, while additionally compensating for
        gravity and ground forces."""
        angles, vel, a = trajectory.get_state(t)
        self._commanded_positions[:] = angles
        self._commanded_velocities[:] = vel

        kin = self._kinematics
        # Compute jacobians
        kin.get_jacobian_end_effector(angles, self._jacobian_ee)
        kin.get_jacobians('com', angles, self._jacobian_coms)

        self.compute_torques(gravity, foot_force)
        self.set_command()

    def get_state_at_time(self, t):
        """Used to determine calculated positions and velocities without
        sending them to the modules."""
        kin = self._kinematics

        velocities = None
        angles = self._feedback_view.position

        stance_mode = self._mode == 'stance'

        if stance_mode:
            kin.solve_inverse_kinematics(self._seed_angles, endeffector_position_objective(self._cmd_stance_xyz), output=angles)
        else:
            self._step.compute_state(t, angles, self._commanded_velocities, None)

        if stance_mode:
            # For stance mode, also compute velocities
            jacobian_ee = self._jacobian_ee
            kin.get_jacobian_end_effector(angles, jacobian_ee)

            try:
                # Note: C++ uses column pivoted householder QR to solve here. Not sure if the additional precision is necessary
                velocities = np.array(solve(jacobian_ee[0:3, :], self._stance_vel_xyz), dtype=float32)
            except LinAlgError as lin:
                velocities = np.zeros(Leg.joint_count, dtype=float32)

        return angles, velocities

    def compute_torques(self, gravity, foot_force):
        """
        TODO: Document
        """
        self._spring_torque[1] = self._spring_shift
        self._spring_torque[1] += self._drag_shift * self._commanded_velocities[1]

        jacobian_ee = self._jacobian_ee
        jacobian_coms = self._jacobian_coms
        jacobian_part = jacobian_ee[0:3, :]
        stance = jacobian_part.T @ -foot_force

        masses = self._masses

        num_bodies = len(masses)
        num_joints = Leg.joint_count
        grav_comp = np.zeros(num_joints, dtype=float64)

        for i in range(num_bodies):
            frame = jacobian_coms[i]
            grav_comp -= frame[0:3, :].T @ gravity * masses[i]

        self._commanded_efforts[:] = grav_comp
        self._commanded_efforts += stance
        self._commanded_efforts += self._spring_torque

    def compute_state(self, t):
        """
        TODO: Document
        """

        kin = self._kinematics
        stance_mode = self._mode == 'stance'

        angles = np.zeros(kin.dof_count)
        if stance_mode:
            kin.solve_inverse_kinematics(self._seed_angles, endeffector_position_objective(self._cmd_stance_xyz), output=angles)
        else:
            self._step.compute_state(t, angles, self._commanded_velocities, None)

        # Compute jacobians
        jacobian_ee = self._jacobian_ee
        jacobian_coms = self._jacobian_coms
        kin.get_jacobian_end_effector(self._feedback_view.position, jacobian_ee)
        kin.get_jacobians('com', self._feedback_view.position, jacobian_coms)

        self._commanded_positions[:] = angles

        if stance_mode:
            # For stance mode, also compute velocities
            try:
                # Note: C++ uses column pivoted householder QR to solve here. Not sure if the additional precision is necessary
                self._commanded_velocities[:] = solve(jacobian_ee[0:3, :], self._stance_vel_xyz)
            except LinAlgError as lin:
                self._commanded_velocities.fill(0.0)

        self._on_state_computed(angles, self._commanded_velocities, self._commanded_efforts, self)

    def update_stance(self, translation_vel, rotate_vel, dt):
        """
        TODO: Document
        """

        # stance_vel_xyz = translation_vel + rotate_vel.cross(cmd_stance_xyz)
        self._stance_vel_xyz[:] = translation_vel + np.cross(rotate_vel, self._cmd_stance_xyz)

        # FIXME: Cut down on allocations here
        self._cmd_stance_xyz += translation_vel * dt
        self._cmd_stance_xyz[:] = rotate_z(rotate_vel[2] * dt) @ rotate_y(rotate_vel[1] * dt) @ rotate_x(rotate_vel[0] * dt) @ self._cmd_stance_xyz

        self._kinematics.get_end_effector(self._feedback_view.position, self._ee_frame)
        np.copyto(self._fbk_stance_xyz, self._ee_frame[0:3, 3])

        # Update home stance to match the current z height
        self._level_home_stance_xyz[2] += translation_vel[2] * dt
        self._home_stance_xyz[:] = rotate_x(0.2 * translation_vel[1]) @ rotate_y(-0.2 * translation_vel[0]) @ self._level_home_stance_xyz

    def start_step(self, t):
        self._step.reset(t, self)
        self._mode = 'flight'

    def update_step(self, t):
        if self._mode != 'flight':
            return
        if self._step.update(t, self):
            np.copyto(self._cmd_stance_xyz, self._step.touch_down)
            # Transition to stance mode
            self._mode = 'stance'

    def get_step_time(self, t):
        return t - self._step.start_time

    def set_command(self, position=True, velocity=True, effort=True):
        """
        NOTE: Pass booleans, not arrays
        """
        view = self._command_view
        view.position = self._commanded_positions if position else None
        view.velocity = self._commanded_velocities if velocity else None
        view.effort = self._commanded_efforts if effort else None

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
        return self._fbk_stance_xyz

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
        return self._mode

    @property
    def step_period(self):
        return self._step.period
