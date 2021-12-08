import numpy as np
from hebi.arm import ArmPlugin

def get_dynamic_comp_efforts(fbk_positions, cmd_positions, cmd_velocities, cmd_accels, robot, dt=1e-3):
  """
  :param fbk_positions:
  :param cmd_positions:
  :param cmd_velocities:
  :param cmd_accels:
  :param robot:
  :param dt:
  :return:
  :rtype:  np.ndarray
  """
  dt_s = dt*dt
  dt_s_inv = 1/dt_s
  cmd_v_dt = cmd_velocities*dt
  cmd_accel_dt = .5*cmd_accels*dt_s

  # get positions at +/- dt
  cmd_positions_last = cmd_positions-cmd_v_dt+cmd_accel_dt
  cmd_positions_next = cmd_positions+cmd_v_dt+cmd_accel_dt

  # Get Forward kinematics for all 3 sets of angles
  cmd_frames_last = [entry[:3, 3] for entry in robot.get_forward_kinematics('CoM', cmd_positions_last)]
  cmd_frames_now = [entry[:3, 3] for entry in robot.get_forward_kinematics('CoM', cmd_positions)]
  cmd_frames_next = [entry[:3, 3] for entry in robot.get_forward_kinematics('CoM', cmd_positions_next)]

  # Build wrench vector and calculate compensatory torques
  efforts = np.zeros(robot.dof_count)
  jacobians = robot.get_jacobians('CoM', fbk_positions)
  masses = robot.masses
  wrench = np.zeros(6)

  for module in range(len(masses)):
    # Calculate XYZ accelerations of the CoM
    lastp = cmd_frames_last[module]
    nowp = cmd_frames_now[module]
    nextp = cmd_frames_next[module]

    accel = ((lastp+nextp)-(2*nowp))*dt_s_inv

    # Set translational part of wrench vector (rotational stays zero)
    wrench[0:3] = accel * masses[module]

    # compEffort = J' * wrench
    efforts += jacobians[module].T @ wrench

  return efforts


class DynamicCompEffortPlugin(ArmPlugin):
    def on_associated(self, arm):
        pass

    def update(self, arm):
        # apply compensation torques for arm dynamics
        if arm.trajectory is not None:
            fbk_p = arm.last_feedback.position
            t_traj = min(arm._last_time - arm._trajectory_start_time, arm.trajectory.duration)
            p, v, a = arm.trajectory.get_state(t_traj)
            dyn_comp_eff = get_dynamic_comp_efforts(fbk_p, p, v, a, arm.robot_model)
            arm.pending_command.effort += dyn_comp_eff


