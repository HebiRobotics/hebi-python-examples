import numpy as np
from .leg import Leg

from hebi.robot_model import endeffector_position_objective
from hebi.trajectory import create_trajectory
from numpy import float32, float64
from numpy.linalg import norm, solve, LinAlgError

# TODO: Parameterize
period = 0.7
overshoot = 0.3
height = 0.04
phase = [0.0, 0.5, 1.0]
max_waypoint_count = 3
# HACK: Hardcoded to be 15 ms. This ensures that the trajectory generator does not loose too much precision
#       from timepoints being too close to one another
ignore_waypoint_threshold = 0.015
replan_stance_vel_threshold = 0.05

reference_time = np.array(phase, dtype=float64) * period

nan = float('nan')


class Step:
    __slots__ = ('_start_time', '_time', '_lift_off_vel',
                 '_lift_up', '_mid_step_1',
                 '_touch_down', '_trajectory',
                 # Workspace for trajectory generation
                 '_waypoint_pos', '_waypoint_vel', '_waypoint_acc',
                 # Starting waypoint
                 '_start_pos', '_start_vel',
                 # Mid waypoint
                 '_mid_pos',
                 # End waypoint
                 '_end_pos', '_end_vel', '_end_jacobian_ee'
                 )

    def __init__(self, start_time, lift_up):
        num_joints = Leg.joint_count

        self._start_time = start_time
        self._time = reference_time.copy()
        self._lift_off_vel = np.zeros(3, dtype=float64)
        self._lift_up = np.array(lift_up, dtype=float64).reshape((3, ))
        self._mid_step_1 = np.zeros(3, dtype=float64)
        self._touch_down = np.zeros(3, dtype=float64)
        self._trajectory = None

        self._waypoint_pos = np.zeros((num_joints, max_waypoint_count), dtype=float64)
        self._waypoint_vel = np.zeros((num_joints, max_waypoint_count), dtype=float64)
        self._waypoint_acc = np.zeros((num_joints, max_waypoint_count), dtype=float64)

        self._start_pos = np.zeros(num_joints, dtype=float64)
        self._start_vel = np.zeros(num_joints, dtype=float64)
        self._mid_pos = np.zeros(num_joints, dtype=float64)
        self._end_pos = np.zeros(num_joints, dtype=float64)
        self._end_vel = np.zeros(num_joints, dtype=float64)
        self._end_jacobian_ee = np.zeros((3, 3), dtype=float64)

    def _replan_step(self, current_stance_vel):
        """Replan the currently generated trajectory, updating the end point to
        have the stance velocity provided.

        The only parameter which changes in this new path is the
        cartesian-space velocity of the leg at the end of the step (the
        "touch down"). The actual cartesian-space position remains the
        same.
        """
        jacobian_part = self._end_jacobian_ee

        time_vector = self._time
        waypoint_pos = self._waypoint_pos
        waypoint_vel = self._waypoint_vel
        waypoint_acc = self._waypoint_acc

        self._end_vel[:] = solve(jacobian_part, current_stance_vel)
        waypoint_vel[:, 2] = self._end_vel

        self._trajectory = create_trajectory(time_vector, waypoint_pos, waypoint_vel, waypoint_acc)

    def reset(self, start_time, leg):
        """Reset the state of this step.

        Use this as opposed to reinstantiating this class.
        """
        self._start_time = start_time
        self._lift_off_vel[:] = leg.stance_vel_xyz

        # Starting point
        self._lift_up[:] = leg.cmd_stance_xyz

        # End point
        self._touch_down[:] = leg.home_stance_xyz - (overshoot * period * self._lift_off_vel)

        # Mid point: Linearly interpolate along the ground
        self._mid_step_1[:] = (self._lift_up + self._touch_down) * 0.5
        # Set z position
        self._mid_step_1[2] += height

        time_vector = self._time
        waypoint_pos = self._waypoint_pos
        waypoint_vel = self._waypoint_vel
        waypoint_acc = self._waypoint_acc
        kin = leg.kinematics
        jacobian_ee = leg._jacobian_ee

        # Initial waypoint

        # Save results from IK, since `_lift_up` is only ever changed on a step reset
        kin.solve_inverse_kinematics(leg.seed_angles, endeffector_position_objective(self._lift_up), output=self._start_pos)
        # Save initial angular velocities for same reason as IK above.
        kin.get_jacobian_end_effector(self._start_pos, jacobian_ee)
        jacobian_part = jacobian_ee[0:3, :]
        self._start_vel[:] = solve(jacobian_part, self._lift_off_vel)

        waypoint_pos[:, 0] = self._start_pos
        waypoint_vel[:, 0] = self._start_vel
        waypoint_acc[:, 0].fill(0.0)
        time_vector[0] = reference_time[0]

        # Mid waypoint
        kin.solve_inverse_kinematics(leg.seed_angles, endeffector_position_objective(self._mid_step_1), output=self._mid_pos)
        waypoint_pos[:, 1] = self._mid_pos
        waypoint_vel[:, 1].fill(nan)
        waypoint_acc[:, 1].fill(nan)
        time_vector[1] = reference_time[1]

        # End waypoint
        kin.solve_inverse_kinematics(leg.seed_angles, endeffector_position_objective(self._touch_down), output=self._end_pos)
        kin.get_jacobian_end_effector(self._end_pos, jacobian_ee)
        jacobian_part = jacobian_ee[0:3, :]
        self._end_vel[:] = solve(jacobian_part, self._lift_off_vel)
        # Note: For each step, the end position (joint angles) does not change throughout the duration of the step.
        # Consequently, neither does the jacobian end effector at the step destination. This can be safely
        # accessed throughout the duration of the step.

        # NOTE: MATLAB implementation _does_ update the end position wrt input stance velocity.
        # If it is desired to track more precisely, then this must be updated on each replan
        # in the function `_replan_step`.
        np.copyto(self._end_jacobian_ee, jacobian_part)

        waypoint_pos[:, 2] = self._end_pos
        waypoint_vel[:, 2] = self._end_vel
        waypoint_acc[:, 2].fill(0.0)
        time_vector[2] = reference_time[2]

        self._trajectory = create_trajectory(time_vector, waypoint_pos, waypoint_vel, waypoint_acc)

    def update(self, t, leg):
        elapsed = t - self._start_time

        if elapsed > period:
            return True
        elif (period - elapsed) < ignore_waypoint_threshold:
            # Close enough to the end. Do not replan.
            return False

        # Check if there has been any change in the stance velocity. If so, re-plan
        current_stance_vel = leg.stance_vel_xyz.copy()
        if norm(current_stance_vel - self._lift_off_vel) > replan_stance_vel_threshold:
            # Replan
            self._replan_step(current_stance_vel)

        return False

    def compute_state(self, t, angles, vels, accels):
        num_joints = Leg.joint_count

        p, v, a = self._trajectory.get_state(t - self._start_time)
        np.copyto(angles, p)
        np.copyto(vels, v)

        # Note: `accels` is not used in C++ code, so we don't use it here for now.
        #np.copyto(accels, a)

    @property
    def start_time(self):
        return self._start_time

    @property
    def touch_down(self):
        return self._touch_down.copy()

    @property
    def period(self):
        return period
