#!/usr/bin/env python3

import hebi
from time import sleep, time, perf_counter
import numpy as np

import os
import sys

# ------------------------------------------------------------------------------
# Add the root folder of the repository to the search path for modules
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path = [root_path] + sys.path
# ------------------------------------------------------------------------------

from util.math_utils import get_grav_comp_efforts
from util.math_utils import gravity_from_quaternion

class ArmParams(object):
    
    def __init__(self, family, moduleNames, hrdf, hasGripper=False, gripperName=None):
        self.family = family
        self.hrdf = hrdf
        self.moduleNames = moduleNames
        self.hasGripper = hasGripper
        self.gripperName = gripperName
            
            

class Gripper(object):

    def __init__(self, family, gripperName):
        self.family = family
        self.name = gripperName

        self.group = hebi.Lookup().get_group_from_names([family], [gripperName])
        if self.group is None:
            print ("Could not find gripper on network! Exiting...")
            sys.exit()

        self.openEffort = 1
        self.closeEffort = -5
        self.state = 0; # 0 is open, 1 is closed

        # set up command structs
        self.cmd = hebi.GroupCommand(1) # only 1 module in the group

    # Loads gains from given gains file    
    def loadGains(self, gains_file):
        try:
            gains_cmd = hebi.GroupCommand(1) # only 1 module in the group
            gains_cmd.read_gains(gains_file)
            if not self.group.send_command_with_acknowledgement(gains_cmd):
                raise RuntimeError('Did not receive acknowledgement from group.')
            print('Successfully read gains from file and sent to gripper.')
        except Exception as e:
            print('Problem reading gains from file or sending to module: {0}'.format(e))
    
    def setState(self, new_state):
        # setState sets gripper to a value between [0 - 1] where 0 
        # is fully open and 1 is fully closed. 'nan' is ignored.
        if new_state < 0 or new_state > 1:
            print("You have tried setting the gripper state to be:")
            print(self.state, new_state)
            print("Gripper state must be between 0 and 1")
        if ~np.isnan(new_state):
            self.cmd.effort = new_state * self.closeEffort + (1-new_state) * self.openEffort
            self.state = new_state

    def close(self):
        self.setState(1)

    def open(self):
        self.setState(0)

    def toggle(self):
        if self.state == 0:
            self.setState(1)
        else:
            self.setState(0)

    def update(self):
        self.group.send_command(self.cmd)


    
class Arm():
    
    def __init__(self, params, lookup = None):
        if lookup is None:
            # Create arm from lookup
            self.lookup = hebi.Lookup()
            sleep(2)
        else:
            self.lookup = lookup

        # Lookup modules for arm
        self.group = self.lookup.get_group_from_names([params.family], params.moduleNames)
        if self.group is None:
            print("Could not find arm on network")
            modules_on_network = [entry for entry in self.lookup.entrylist]
            if len(modules_on_network) == 0:
                print("No modules on network")
            else:
                print("modules on network:")
                for entry in modules_on_network:
                    print(entry)

            raise RuntimeError()

        # Create robot model
        try:
            self.model = hebi.robot_model.import_from_hrdf(params.hrdf)
        except Exception as e:
            print('Could not load HRDF: {0}'.format(e))
            sys.exit()
        
        # Create Gripper, if exists
        if params.hasGripper:
            self.gripper = Gripper(params.family, params.gripperName)
        else:
            self.gripper = None


        # Create comand and feedback
        self.cmd = hebi.GroupCommand(self.group.size)
        self.fbk = hebi.GroupFeedback(self.group.size)
        self.prev_fbk = self.fbk
        self.group.get_next_feedback(reuse_fbk=self.fbk)
        
        # Zero our initial comand setting vars
        self.pos_cmd = self.fbk.position
        self.vel_cmd = np.zeros(self.group.size)
        self.accel_cmd = np.zeros(self.group.size)
        self.eff_cmd = np.zeros(self.group.size)
        
        # Setup gravity vector for grav comp
        self.gravity_vec = [0, 0, 1]
        gravity_from_quaternion(self.fbk.orientation[0], output=self.gravity_vec)
        self.gravity_vec = np.array(self.gravity_vec)
        
        # Zero times
        self.time_now = time()
        self.traj_start_time = self.time_now
        self.duration = 0
        self.t_traj = 0
        
        self.set_goal_first = True
        self.at_goal = True
        self.goal = "grav comp"
        
    
    # Loads gains from given gains file    
    def loadGains(self, gains_file):
        try:
            gains_cmd = hebi.GroupCommand(self.group.size)
            gains_cmd.read_gains(gains_file)
            if not self.group.send_command_with_acknowledgement(gains_cmd):
                raise RuntimeError('Did not receive ack from group.')
            print('Successfully read gains from file and sent to arm.')
        except Exception as e:
            print('Problem reading gains from file or sending to arm: {0}'.format(e))
    
    
    # Updates feedback and generates the basic command for this timestep
    def update(self):
        self.prev_fbk = list(self.fbk)
        self.group.get_next_feedback(reuse_fbk=self.fbk)
        self.time_now = perf_counter()
        if not self.at_goal:
            self.t_traj = self.time_now - self.traj_start_time
            if self.time_now > (self.traj_start_time + self.duration):
                self.at_goal = True
        
        if self.gripper != None:
            self.gripper.update()
        
        # Create command
        if self.goal == "grav comp":
            # Calculate required torques to negate gravity at current position
            nan_array = np.empty(self.group.size)
            nan_array[:] = np.nan
            self.cmd.position = nan_array
            self.cmd.velocity = nan_array
            self.cmd.effort = get_grav_comp_efforts(self.model, self.fbk.position, -self.gravity_vec)
        elif not self.at_goal:
            # Move to target position
            self.pos_cmd, self.vel_cmd, self.accel_cmd = self.trajectory.get_state(self.t_traj)
            self.cmd.position = self.pos_cmd
            self.cmd.velocity = self.vel_cmd
            self.cmd.effort = get_grav_comp_efforts(self.model, self.fbk.position, -self.gravity_vec)
        else:
            # Stay at current position
            self.cmd.position = self.fbk.position_command
        return 
    
    
    def send(self):
        # Send command
        self.group.send_command(self.cmd)
        return

    
    def createMotionArray(self, size, prev_cmd, array=None, flow=None):
        # Helper function to create arrays for things like waypoints, velocities, and accelerations that are used in creating a trajectory
        # Note flow overrides any array passed (used for vel and accel, should never pass flow and positions)
        mArray = np.empty((self.group.size, size+1))
        mArray[:, 0] = prev_cmd
        if flow == None:
            for i in range(size):
                if array == None:
                    mArray[:, i+1] = np.zeros(self.group.size)
                else:
                    mArray[:, i+1] = array[i]
        else:
            nan_array = np.empty(self.group.size)
            nan_array[:] = np.nan
            for i in range(size):
                if flow[i]:
                    mArray[:, i+1] = nan_array
                else:
                    mArray[:, i+1] = np.zeros(self.group.size)
        
        return mArray
    
    
    def createGoal(self, position, duration=None, velocity=None, accel=None, flow=None):
        # Create trajectory to target position
        # Position should be an array of joint positions
        # Duration should be an array of durations (if passed)
        # Velocity should be an array of joint velocities of the same length as the amount of positions (if passed)
        # Accel should be an array of joint accelerations of the same length as the amount of positions (if passed)
        # Flow ahould be an array of True/False of the same length as the amount of positions (if passed)
        # Note flow overrides any velocities or accelerations passed
        """
        EX:
            3dof arm with a 2 point trajectory
            position = [[pos1, pos2, pos3], [pos4, pos5, pos6]]
            duration = [time1, time2]
            velocity = [[vel1, vel2, vel3], [vel4, vel5, vel6]]
            accel = [[accel1, accel2, accel3], [accel4, accel5, accel6]]
            flow = [bool1, bool2]
        """
        self.at_goal = False
        
        if self.goal == "grav comp":
            self.goal = "position"
            self.pos_cmd = self.fbk.position
            self.vel_cmd = self.fbk.velocity
            self.accel_cmd = np.zeros(self.group.size)
        
        waypoints = self.createMotionArray(len(position), self.pos_cmd, array=position)
        
        # If flow is given it overwrites vel and accels given
        velocities = self.createMotionArray(len(position), self.vel_cmd, array=velocity, flow=flow)
        accels = self.createMotionArray(len(position), self.accel_cmd, array=accel, flow=flow)
        
        # Create time vector from durations given
        time_vector = []
        if duration == None:
            time_vector.append(0)
            for i in range(len(position)):
                time_vector.append(5+time_vector[i])
        else:
            time_vector.append(0)
            for i in range(len(position)):
                time_vector.append(duration[i]+time_vector[i])
        
        # Create trajectory
        self.trajectory_plan = hebi.trajectory.create_trajectory(time_vector, waypoints, velocities, accels)
        return self.trajectory_plan
    
    
    def setGoal(self, goal=None):
        # If no goal is passed we use the last goal created by createGoal, otherwise set trajectory passed as goal and zero times
        # Goal passed should be a trajectory not a target
        if goal == None:
            self.trajectory = self.trajectory_plan
        else:
            self.trajectory = goal
        self.duration = self.trajectory.duration
        self.traj_start_time = self.time_now
        return
    
      
    def cancelGoal(self):
        # Return to grav comp mode
        self.goal = "grav comp"
        return
        
      
    def goalProgress(self):
        # Return progress of the arm through the trajectory
        if not self.at_goal:
            return self.t_traj/self.duration
        else:
            # No current goal
            return 0
        
        
        