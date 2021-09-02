#!/usr/bin/env python3

import hebi
import numpy as np
import os
import sys


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


from hebi.trajectory import create_trajectory
from hebi.robot_model import endeffector_position_objective, endeffector_so3_objective
import mobile_io as mbio
from util.math_utils import get_grav_comp_efforts, get_dynamic_comp_efforts, quat2rot, rotate_y
from util.arm import setup_arm_params
from time import sleep, perf_counter, time


enable_logging = False
enable_effort_comp = True

# Mobile Device Setup
phone_family = 'HEBI'
phone_name = "Cal's iPhone"

reset_pose_button = 0
quit_demo_button = 7
translation_scale_slider = 2
grip_force_slider = 5

abort_flag = False
lookup = hebi.Lookup()
sleep(2)


print('Waiting for Mobile IO device to come online...')
m = mbio.MobileIO(phone_family, phone_name)
state = m.getState()
m.setButtonMode(1, 1)
# Arm Setup
arm_name = '6-DoF'
arm_family = 'Example Arm'
has_gas_spring = False  # If you attach a gas spring to the shoulder for extra payload, set this to True.

group, kin, params = setup_arm_params(arm_name, arm_family, has_gas_spring, lookup=lookup)

fbk = hebi.GroupFeedback(group.size)
ik_seed_pos = params.ik_seed_pos
effort_offset = params.effort_offset
gravity_vec = params.gravity_vec
local_dir = params.local_dir

if params.has_gripper:
    gripper_group = lookup.get_group_from_names(arm_family, 'Spool')

    if gripper_group is None:
        raise RuntimeError("Cannot create gripper group.")

    gripper_group.send_command(params.gripper_gains)
    grip_force_scale = 0.5 * (params.gripper_open_effort - params.gripper_close_effort)
    grip_force_shift = np.mean([params.gripper_open_effort, params.gripper_close_effort])
    gripper_cmd = hebi.GroupCommand(1)

arm_dof_count = kin.dof_count

# Min move time for 'small' movements.
arm_traj_min_duration = 1.0

print('Arm end-effector is now following the mobile device pose.')
print('The control interface has the following commands:')
print('  B1 - Reset/re-align poses.')
print('       This takes the arm to home and aligns with mobile device.')
print('  A3 - Scale down the translation commands to the arm.')
print('       Sliding all the way down means the end-effector only rotates.')
print('  A6 - Control the gripper (if the arm has gripper).')
print('       Sliding down closes the gripper, sliding up opens.')
print('  B8 - Quits the demo.')

cmd = hebi.GroupCommand(group.size)

# Move to current coordinates
xyz_target_init = np.asarray([0.5, 0.0, 0.1])
rot_mat_target_init = rotate_y(np.pi)


def get_ik(xyz_target, rot_target, ik_seed):
    # Helper function to rebuild the IK solver with the appropriate objective functions
    return kin.solve_inverse_kinematics(ik_seed_pos, endeffector_position_objective(xyz_target), endeffector_so3_objective(rot_target))


counter = 0
# Startup
while not abort_flag:
    # Update mobile io state
    state = m.getState()
    print("counter: " + str(counter))
    print("state: " + str(state))
    counter += 1
    if counter % 100 == 0:
        print("counter: " + str(counter))
    group.get_next_feedback(reuse_fbk=fbk)

    cmd.position = None
    cmd.velocity = None
    cmd.effort = None

    xyz_scale = np.asarray([1, 1, 2])

    # Start background logging
    if enable_logging:
        group.start_log(os.path.join(local_dir, 'logs'), mkdirs=True)
        #phone_group.start_log(os.path.join(local_dir, 'logs'), mkdirs=True)

    ik_pos = get_ik(xyz_target_init, rot_mat_target_init, ik_seed_pos)

    arm_traj = create_trajectory([0, arm_traj_min_duration], np.array([fbk.position, ik_pos]).T)
    fbk_time = fbk.receive_time
    t0 = fbk_time.min()
    t_start = time()
    t = 0
    #t = 0
    print("check 1")
    while t < arm_traj.duration:

        group.get_next_feedback(reuse_fbk=fbk)
        t = time() - t_start
        #t = min((fbk_time - t0).min(), arm_traj.duration)

        pos, vel, accel = arm_traj.get_state(t)
        cmd.position = pos
        cmd.velocity = vel
        if enable_effort_comp:
            fbk_position = fbk.position
            dynamics_comp = get_dynamic_comp_efforts(fbk_position, pos, vel, accel, kin)
            grav_comp = get_grav_comp_efforts(kin, fbk_position, gravity_vec)
            cmd.effort = dynamics_comp + grav_comp + effort_offset

        group.send_command(cmd)

    # Grab initial pose
    fbk_mobile = m.fbk

    q = fbk_mobile.ar_orientation[0]
    R_init = quat2rot(q)

    xyz_init = fbk_mobile.ar_position[0]
    xyz_phone_new = xyz_init

    end_velocities = np.zeros(arm_dof_count)
    end_accels = np.zeros(arm_dof_count)

    phone_fbk_timer = perf_counter()

    time_last = t0
    arm_trajStartTime = t0

    first_run = True

    print("check 2")
    while not abort_flag:
        # Update mobile io state
        state = m.getState()
        print("check 3")
        print("state: ")
        print(state)
        group.get_next_feedback(reuse_fbk=fbk)

        time_now = fbk.transmit_time
        dt = time_now - time_last
        time_last = fbk.transmit_time

        # Reset the Command Struct
        cmd.effort = None
        cmd.position = None
        cmd.velocity = None

        # Check for restart command
        if state[0][reset_pose_button] == 0:

            break

        # Check for quit command
        if state[0][quit_demo_button] == 1:

            abort_flag = True
            break

        if params.has_gripper:
            gripper_cmd.effort = grip_force_scale * state[1][grip_force_slider] + grip_force_shift
            gripper_group.send_command(gripper_cmd)

        # Parameter to limit XYZ Translation of the arm if a slider is pulled down.
        # Pulling all the way down resets translation.
        phone_control_scale = state[1][translation_scale_slider]
        if phone_control_scale < 0:
            xyz_init = xyz_phone_new

        # Pose Information for Arm Control
        xyz_phone_new = fbk_mobile.ar_position[0]
        xyz_target = xyz_target_init + (np.multiply(phone_control_scale * xyz_scale, R_init.T @ (xyz_phone_new - xyz_init)))
        q = fbk_mobile.ar_orientation[0]
        rot_mat_target = R_init.T @ quat2rot(q) @ rot_mat_target_init

        # Get state of current trajectory
        if first_run:
            pos = fbk.position_command
            vel = end_velocities
            accel = end_accels
            first_run = False
        else:
            t = (time_now - arm_trajStartTime)[0]
            pos, vel, accel = arm_traj.get_state(t)

        cmd.position = pos
        cmd.velocity = vel

        fbk_position = fbk.position

        if enable_effort_comp:

            dynamics_comp = get_dynamic_comp_efforts(fbk_position, pos, vel, accel, kin)
            grav_comp = get_grav_comp_efforts(kin, fbk_position, gravity_vec)
            cmd.effort = dynamics_comp + grav_comp + effort_offset

        # Force elbow up config
        seed_pos_ik = pos
        seed_pos_ik[3] = abs(seed_pos_ik[3])

        # Find target using inverse kinematics
        ik_pos = get_ik(xyz_target, rot_mat_target, seed_pos_ik)

        # Start new trajectory at the current state
        phone_hz = 10
        phone_period = 1 / phone_hz

        current_time = perf_counter()

        if (current_time - phone_fbk_timer) > phone_period:
            arm_trajStartTime = time_now
            phone_fbk_timer = current_time
            a = [0, arm_traj_min_duration]
            b = np.array([pos, ik_pos]).T
            c = np.array([vel, end_velocities]).T
            d = np.array([accel, end_accels]).T
            arm_traj = create_trajectory(a, b, c, d)

        print("check 4")
        # Send to robot
        #group.send_command(cmd)


if enable_logging:
    hebi_log = group.stop_log()

    # Plot tracking / error from the joints in the arm. Note that there
    # will not by any 'error' in tracking for position and velocity, since
    # this example only commands effort.
    hebi.util.plot_logs(hebi_log, 'position')
    hebi.util.plot_logs(hebi_log, 'velocity')
    hebi.util.plot_logs(hebi_log, 'effort')

    # Put more plotting code here
