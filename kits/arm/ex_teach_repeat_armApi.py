# -*- coding: utf-8 -*-
import arm
import mobile_io as mbio


# Set up arm
family_name = "Arm"
module_names = ("J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3")
hrdf = "hrdf/6-DoF_arm.hrdf"
p = arm.ArmParams(family_name, module_names, hrdf)
a = arm.Arm(p)

# Set up our mobile io interface
print('Waiting for Mobile IO device to come online...')
phone_family = "HEBI"
phone_name = "Cal's iPhone"
m = mbio.MobileIO(phone_family, phone_name)
state = m.getState()

abort_flag = False

waypoints = []
run_mode = "training"
curr_waypoint_num = 0

while not abort_flag:
    # Update arm and mobile io
    a.update()
    prev_state = state
    state = m.getState()
    diff = m.getDiff(prev_state, state)
    
    # Check for quit
    if diff[7] == "rising":
        abort_flag = True
        break
    
    if run_mode == "training":
        # B1 add waypoint
        if diff[0] == "rising":
            print("Waypoint added")
            waypoints.append(a.fbk.position)
        
        # B2 toggle training/playback
        if diff[1] == "rising":
            # Check for more than 2 waypoints
            if len(waypoints) > 1:
                run_mode = "playback"
                curr_waypoint_num = 0
                a.send()
                continue
            else:
                print("At least two waypoints are needed")
        
        # B4 clear waypoints
        if diff[4] == "rising":
            print("Waypoints cleared")
            waypoints = []
            
    if run_mode == "playback":
        # B2 toggle training/playback
        if diff[1] == "rising":
            run_mode = "training"
            a.cancelGoal()
        
        # When at a waypoint, go to the next one
        if a.at_goal:
            target = waypoints[curr_waypoint_num]
            a.setGoal(4, target)
            if curr_waypoint_num < len(waypoints) - 1:
                curr_waypoint_num += 1
            else:
                curr_waypoint_num = 0
    
    
    a.send()
        
        
        
        
        