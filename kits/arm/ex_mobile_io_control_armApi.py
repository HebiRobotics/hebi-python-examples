#!/usr/bin/env python3

import numpy as np
from time import sleep

import arm
import hebi
from hebi.util import create_mobile_io

# Set up arm
family_name  = "Example Arm"
module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
hrdf = "hrdf/A-2085-06.hrdf"
p = arm.ArmParams(family_name, module_names, hrdf)
a = arm.Arm(p)

# Mobile device setup
phone_family = 'HEBI'
phone_name   = "mobileIO"

lookup = hebi.Lookup()
sleep(2)

print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.update()

abort_flag = False

while not abort_flag:
  # Update arm
  a.update()
  # Update MobileIO state
  if not m.update():
    print("Failed to get feedback from MobileIO")
    continue

  # B8 quit demo
  if m.get_button_diff(8) == 3: # "ToOn"
    abort_flag = True
    break

  # B1 point 1
  if m.get_button_diff(1) == 3: # "ToOn"
    positions = (0, 0, 0, 0, 0, 0)
    a.createGoal([positions], duration=[4])
    a.setGoal()

  # B2 point 2    
  if m.get_button_diff(2) == 3: # "ToOn"
    positions = np.asarray([np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, np.pi/4, 0])
    a.createGoal([positions], duration=[4])
    a.setGoal()

  # B3 point 3
  if m.get_button_diff(3) == 3: # "ToOn"
    positions = np.asarray([-np.pi/4, np.pi/3, 2*np.pi/3, np.pi/3, 3*np.pi/4, 0])
    a.createGoal([positions], duration=[4])
    a.setGoal()

  # B6 grav comp    
  if m.get_button_diff(6) == 3: # "ToOn"
    a.cancelGoal()

  # Send comands to arm    
  a.send()
