# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 09:58:36 2020

@author: hebi
state machine

states: startup, standby, control

startup
    move to starting position
    
standby
    wait for imput and hold at starting position
    
control
    follow phone imput and move arm
        find phone pos
        find phone orientation
        plot course from current arm pos to mobile pos and orientation

"""

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
from hebi.robot_model import endeffector_position_objective, endeffector_so3_objective, custom_objective
from util.math_utils import get_grav_comp_efforts, get_dynamic_comp_efforts, quat2rot, rotate_y
from util.arm import setup_arm_params
from time import sleep, perf_counter, time
from util import math_utils

import mobile_io as mbio


run_mode = "startup"

enable_logging = False
enable_effort_comp = True


# Mobile device setup
phone_family = 'HEBI'
phone_name = "Cal's iPhone"

control_mode_toggle = 0
quit_demo_button = 7
translation_scale_slider = 2
grip_force_slider = 5

print('Waiting for Mobile IO device to come online...')
m = mbio.MobileIO(phone_family, phone_name)
state = m.getState()
m.setButtonMode(1, 1)

abort_flag = False
lookup = hebi.Lookup()
sleep(2)

# Arm Setup
arm_name = '6-DoF'
arm_family = 'Example Arm'
has_gas_spring = False # If you attach a gas spring to the shoulder for extra payload, set this to True.

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

# Startup coordinates
xyz_target_init = np.asarray([0.5, 0.0, 0.1])
rot_mat_target_init = rotate_y(np.pi)


mobile_pos_offset = [0, 0, 0]

log = []

def logger_cb(positions, error, user_data):
    global log
    log.append((positions, error))
    return 0.0


def get_ik(xyz_target, rot_target, ik_seed):
    # Helper function to rebuild the IK solver with the appropriate objective functions
    """
    print("xyz target: ")
    print(xyz_target)
    print("ik seed")
    print(ik_seed)
    global log
    log = []
    #logger = custom_objective(1, logger_cb)
    """
    return kin.solve_inverse_kinematics(ik_seed, endeffector_position_objective(xyz_target), endeffector_so3_objective(rot_target))
    #return kin.solve_inverse_kinematics(ik_seed, endeffector_position_objective(xyz_target), logger)

fbk_time = fbk.receive_time
t0 = fbk_time.min()
first_run = True
end_velocities = np.zeros(arm_dof_count)
end_accels = np.zeros(arm_dof_count)
t1 = 0
arm_trajStartTime = t0
phone_fbk_timer = perf_counter()

# Main run loop
while not abort_flag:
    # Update mobile io state
    state = m.getState()
    fbk_mobile = m.fbk
    
    # Update arm state
    group.get_next_feedback(reuse_fbk=fbk)
    
    
    # Check for abort
    if state[0][quit_demo_button] == 1:
        abort_flag = True
        # Set led to red
        m.setLedColor("red")
        continue
    
    # Check run mode
    if run_mode == "startup":
        # Move to starting pos
        print("Starting up")
        # Set led to yellow
        m.setLedColor("yellow")
        
        joint_targets = get_ik(xyz_target_init, rot_mat_target_init, ik_seed_pos)
        waypoints = np.empty((group.size, 2))
        waypoints[:, 0] = fbk.position
        waypoints[:, 1] = joint_targets
        time_vector = [0, 5]  # Seconds for the motion - do this slowly
        trajectory = hebi.trajectory.create_trajectory(time_vector, waypoints)
        ik_seed_pos = fbk.position
        
        duration = trajectory.duration
        start = time()
        t = time() - start
        
        while t < duration:
            state = m.getState()
            # Check for abort
            if state[0][quit_demo_button] == 1:
                abort_flag = True
                # Set led to red
                m.setLedColor("red")
                break
            # Get feedback and update the timer
            group.get_next_feedback(reuse_fbk=fbk)
            t = time() - start
        
            # Get new commands from the trajectory
            pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
        
            # Calculate commanded efforts to assist with tracking the trajectory.
            # Gravity Compensation uses knowledge of the arm's kinematics and mass to
            # compensate for the weight of the arm. Dynamic Compensation uses the
            # kinematics and mass to compensate for the commanded accelerations of the arm.
            eff_cmd = math_utils.get_grav_comp_efforts(kin, fbk.position, [0, 0, 1])
            # NOTE: dynamic compensation effort computation has not yet been added to the APIs
        
            # Fill in the command and send commands to the arm
            cmd.position = pos_cmd
            cmd.velocity = vel_cmd
            cmd.effort = eff_cmd
            group.send_command(cmd)
            print(pos_cmd)
        
        # Update run mode
        run_mode = "standby"
        print("Standing by")
        
        
    elif run_mode == "standby":
        # Hold pos and wait for input
        
        # Set led to green
        m.setLedColor("green")
        # Check if mode toggle requested
        if state[0][control_mode_toggle] == 1:
            # Change run mode
            run_mode = "control"
            print("Following phone")
            # Set phone zero pos
            mobile_pos_offset = xyz_target_init - fbk_mobile.ar_position[0]
            continue
        # Hold arm in place
        joint_targets = get_ik(xyz_target_init, rot_mat_target_init, params.ik_seed_pos)
        cmd.position = joint_targets
        group.send_command(cmd)
    
    elif run_mode == "control":
        # Follow phones movement
        
        # Set led to blue
        m.setLedColor("blue")
        # Check if mode toggle requested
        if state[0][control_mode_toggle] == 0:
            # Change run mode
            run_mode = "startup"
            continue
        
        
        time_now = fbk.transmit_time

        
        # Get state of current trajectory
        if first_run:
            pos_cmd = fbk.position_command
            vel_cmd = end_velocities
            accel_cmd = end_accels
            first_run = False
        else:
            t1 = (time_now - arm_trajStartTime)[0]
            #print("t1: " + str(t1))
            pos_cmd, vel_cmd, accel_cmd = trajectory.get_state(t1)
        
        if enable_effort_comp:
            dynamics_comp = get_dynamic_comp_efforts(fbk.position, pos_cmd, vel_cmd, accel_cmd, kin)
            grav_comp = get_grav_comp_efforts(kin, fbk.position, gravity_vec)
            cmd.effort = dynamics_comp + grav_comp + effort_offset
        """
        t = time()
        pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(time()-t)
        eff_cmd = math_utils.get_grav_comp_efforts(kin, fbk.position, [0, 0, 1])
        """
        cmd.position = pos_cmd
        cmd.velocity = vel_cmd
        cmd.effort = eff_cmd
        
        # Start new trajectory at the current state        
        phone_hz = 50
        phone_period = 1 / phone_hz
        
        current_time = perf_counter()
        
        if (current_time - phone_fbk_timer) > phone_period:
            print("new traj")
            arm_trajStartTime = time_now
            phone_fbk_timer = current_time
            
            # Find current phone pos
            phone_target_xyz = fbk_mobile.ar_position[0] + mobile_pos_offset
            print("phone target")
            print(phone_target_xyz)
            #phone_target_xyz = np.asarray([0.5, 0.0, 0.3])
            joint_targets = get_ik(phone_target_xyz, rot_mat_target_init, fbk.position)
            
            waypoints = np.empty((group.size, 2))
            waypoints[:, 0] = pos_cmd
            waypoints[:, 1] = joint_targets
            time_vector = [0, 1]
            velocities = np.empty((group.size, 2))
            velocities[:, 0] = vel_cmd
            velocities[:, 1] = end_velocities
            accels = np.empty((group.size, 2))
            accels[:, 0] = accel_cmd
            accels[:, 1] = end_accels
            trajectory = hebi.trajectory.create_trajectory(time_vector, waypoints, velocities, accels)
        
        
        group.send_command(cmd)
        
        array = np.zeros(6)
        fbk.get_position_command(array)
        print(pos_cmd)
        #print(fbk.position)
        
