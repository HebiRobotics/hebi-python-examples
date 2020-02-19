# -*- coding: utf-8 -*-


import hebi
import numpy as np
import os
import sys


# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------


from hebi.robot_model import endeffector_position_objective
from util.math_utils import get_grav_comp_efforts, get_dynamic_comp_efforts
from util.arm import setup_arm_params
from time import sleep, perf_counter, time
from util import math_utils
from matplotlib import pyplot as plt

import mobile_io as mbio


run_mode = "startup"

enable_logging = True
enable_effort_comp = True


# Mobile device setup
phone_family = 'HEBI'
phone_name = "Cal's iPhone"

control_mode_toggle = 0
quit_demo_button = 7


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

arm_dof_count = kin.dof_count
                             
print('Arm end-effector is now following the mobile device pose.')
print('The control interface has the following commands:')
print('  B1 - Reset/re-align poses.')
print('       This takes the arm to home and aligns with mobile device.')
print('  B8 - Quits the demo.')

cmd = hebi.GroupCommand(group.size)

# Startup coordinates
xyz_target_init = np.asarray([0.5, 0.0, 0.1])

mobile_pos_offset = [0, 0, 0]


def get_ik(xyz_target, ik_seed):
    # Helper function to rebuild the IK solver with the appropriate objective functions
    return kin.solve_inverse_kinematics(ik_seed, endeffector_position_objective(xyz_target))

fbk_time = fbk.receive_time
t0 = fbk_time.min()
first_run = True
end_velocities = np.zeros(arm_dof_count)
end_accels = np.zeros(arm_dof_count)
t1 = 0
arm_trajStartTime = t0
phone_fbk_timer = perf_counter()


if enable_logging:
    group.start_log("logs", mkdirs=True)


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
        
        joint_targets = get_ik(xyz_target_init, params.ik_seed_pos)
        waypoints = np.empty((group.size, 2))
        waypoints[:, 0] = fbk.position
        waypoints[:, 1] = joint_targets
        time_vector = [0, 5]  # Seconds for the motion - do this slowly
        trajectory = hebi.trajectory.create_trajectory(time_vector, waypoints)
        #ik_seed_pos = fbk.position
        
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
            #print(pos_cmd)
        
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
            # Reset timers
            fbk_time = fbk.receive_time
            t0 = fbk_time.min()
            first_run = True
            t1 = 0
            arm_trajStartTime = t0
            phone_fbk_timer = perf_counter()
            continue
        # Hold arm in place
        joint_targets = get_ik(xyz_target_init, params.ik_seed_pos)
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
        
        # update time
        time_now = fbk.receive_time

        # Get state of current trajectory
        if first_run:
            pos_cmd = fbk.position_command
            vel_cmd = end_velocities
            accel_cmd = end_accels
            
        else:
            t1 = (time_now - arm_trajStartTime)[0]
            pos_cmd, vel_cmd, accel_cmd = trajectory.get_state(t1)
        
        if enable_effort_comp:
            dynamics_comp = get_dynamic_comp_efforts(fbk.position, pos_cmd, vel_cmd, accel_cmd, kin)
            grav_comp = get_grav_comp_efforts(kin, fbk.position, gravity_vec)
            cmd.effort = dynamics_comp + grav_comp + effort_offset
            

        cmd.position = pos_cmd
        cmd.velocity = vel_cmd
        cmd.effort = eff_cmd
        
        # Start new trajectory at the current state        
        phone_hz = 50
        phone_period = 1 / phone_hz
        
        current_time = perf_counter()
        
        if ((current_time - phone_fbk_timer) > phone_period) or first_run:
            #print("new traj")
            arm_trajStartTime = time_now
            phone_fbk_timer = current_time
            
            # Find current phone pos
            phone_target_xyz = fbk_mobile.ar_position[0] + mobile_pos_offset
            joint_targets = get_ik(phone_target_xyz, fbk.position)
            
            waypoints = np.empty((group.size, 2))
            waypoints[:, 0] = pos_cmd
            waypoints[:, 1] = joint_targets
            
            velocities = np.empty((group.size, 2))
            velocities[:, 0] = vel_cmd
            velocities[:, 1] = end_velocities
            
            accels = np.empty((group.size, 2))
            accels[:, 0] = accel_cmd
            accels[:, 1] = end_accels
            
            time_vector = [0, 1]
            
            trajectory = hebi.trajectory.create_trajectory(time_vector, waypoints, velocities, accels)
            
            first_run = False
        
        group.send_command(cmd)
        



if enable_logging:        
    # Stop logging
    log_file = group.stop_log()
    
    time = []
    position = []
    velocity = []
    effort = []
    # iterate through log
    for entry in log_file.feedback_iterate:
        time.append(entry.transmit_time)
        position.append(entry.position)
        velocity.append(entry.velocity)
        effort.append(entry.effort)
    
    
    # Offline Visualization
    # Plot the logged position feedback
    plt.figure(101)
    plt.plot(time, position)
    plt.title('Position')
    plt.xlabel('time (sec)')
    plt.ylabel('position (rad)')
    plt.grid(True)
    
    # Plot the logged velocity feedback
    plt.figure(102)
    plt.plot(time, velocity)
    plt.title('Velocity')
    plt.xlabel('time (sec)')
    plt.ylabel('velocity (rad/sec)')
    plt.grid(True)
    
    # Plot the logged effort feedback
    plt.figure(103)
    plt.plot(time, effort)
    plt.title('Effort')
    plt.xlabel('time (sec)')
    plt.ylabel('effort (N*m)')
    plt.grid(True)
            
