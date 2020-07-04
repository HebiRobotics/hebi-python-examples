#!/usr/bin/env python3

import numpy as np

import arm
from hebi.util import create_mobile_io

# Set up arm
family_name  = "Arm"
module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
hrdf = "hrdf/6-DoF_arm.hrdf"
p = arm.ArmParams(family_name, module_names, hrdf)
a = arm.Arm(p)

# Mobile device setup
phone_family = 'HEBI'
phone_name   = "mobileIO"

lookup = hebi.Lookup()
sleep(2)

print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
state = m.state

abort_flag = False

while not abort_flag:
  # Update arm
  a.update()

  # Update mobile io state
  state = m.state
  mobile_io_diff = m.getDiff(prev_state, state)

  # B8 quit demo
  if mobile_io_diff[7] == "rising":
    abort_flag = True
    break

  # B1 point 1
  if mobile_io_diff[0] == "rising":
    positions = (0, 0, 0, 0, 0, 0)
    a.createGoal([positions], durration=[4])
    a.setGoal()

  # B2 point 2    
  if mobile_io_diff[1] == "rising":
    positions = np.asarray([np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, np.pi/4, 0])
    a.createGoal([positions], durration=[4])
    a.setGoal()

  # B3 point 3
  if mobile_io_diff[2] == "rising":
    positions = np.asarray([-np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, 3*np.pi/4, 0])
    a.createGoal([positions], durration=[4])
    a.setGoal()

  # B6 grav comp    
  if mobile_io_diff[5] == "rising":
    a.cancelGoal()

  # Send comands to arm    
  a.send()
