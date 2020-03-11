# -*- coding: utf-8 -*-
import numpy as np

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

while not abort_flag:
    # Update arm
    a.update()
    
    # Update mobile io state
    prev_state = state
    state = m.getState()
    mobile_io_diff = m.getDiff(prev_state, state)
    
    # B8 quit demo
    if mobile_io_diff[7] == "rising":
        abort_flag = True
        break
    
    # B1 point 1
    if mobile_io_diff[0] == "rising":
        positions = (0, 0, 0, 0, 0, 0)
        a.setGoal([positions], durration=[4])
    
    # B2 point 2    
    if mobile_io_diff[1] == "rising":
        positions = np.asarray([np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, np.pi/4, 0])
        a.setGoal([positions], durration=[4])
    
    # B3 point 3
    if mobile_io_diff[2] == "rising":
        positions = np.asarray([-np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, 3*np.pi/4, 0])
        a.setGoal([positions], durration=[4])
    
    # B6 grav comp    
    if mobile_io_diff[5] == "rising":
        a.cancelGoal()
    
    # Send comands to arm    
    a.send()
        
        
    