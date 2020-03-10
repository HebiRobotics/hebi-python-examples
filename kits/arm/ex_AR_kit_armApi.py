# -*- coding: utf-8 -*-
import numpy as np
import threading

# Add the root folder of the repository to the search path for modules
import os, sys
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path = [root_path] + sys.path

from hebi.robot_model import endeffector_position_objective

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
m.setButtonMode(1, 1)
state = m.getState()
fbk_mobile = m.fbk
diff = ["", "", "", "", "", "", "", ""]

control_mode_toggle = 0
quit_demo_button = 7

abort_flag = False
run_mode = "startup"

def getMobileState(quit_demo_button):
    global state
    global fbk_mobile
    global diff
    global abort_flag
    global run_mode
    global mobile_pos_offset
    m.setLedColor("yellow")
    while not diff[quit_demo_button] == "rising":
        prev_state = state
        state = m.getState()
        diff = m.getDiff(prev_state, state)
        fbk_mobile = m.fbk
        if diff[0] == "falling":
            run_mode = "startup"
            m.setLedColor("yellow")
        if run_mode == "standby":
            m.setLedColor("green")
        if diff[0] == "rising" and run_mode == "standby":
            m.setLedColor("blue")
            run_mode = "control"
            mobile_pos_offset = xyz_target_init - fbk_mobile.ar_position[0]
            
    m.setLedColor("red")
    abort_flag = True
    return

t1 = threading.Thread(target=getMobileState, args=(quit_demo_button,))
t1.start()

def get_ik(xyz_target, ik_seed):
    # Helper function to rebuild the IK solver with the appropriate objective functions
    return a.model.solve_inverse_kinematics(ik_seed, endeffector_position_objective(xyz_target))

# Startup coordinates
xyz_target_init = np.asarray([0.5, 0.0, 0.1])
ik_seed_pos = np.asarray([0.01, 1.0, 2.5, 1.5, -1.5, 0.01])
mobile_pos_offset = [0, 0, 0]


# Main run loop
while not abort_flag:
    a.update()
    
    if run_mode == "startup":
        joint_targets = get_ik(xyz_target_init, ik_seed_pos)
        a.setGoal(5, joint_targets)
        run_mode = "moving to start pos"
    
    if run_mode == "moving to start pos":
        if a.at_goal:
            run_mode = "standby"
    
    if run_mode == "standby":
        a.send()
        continue
    
    if run_mode == "control":
        phone_target_xyz = fbk_mobile.ar_position[0] + mobile_pos_offset
        joint_targets = get_ik(phone_target_xyz, a.fbk.position)
        a.setGoal(1, joint_targets)
    
    a.send()