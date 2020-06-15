#!/usr/bin/env python3

from time import time
import arm

# Arm setup
family_name = "Arm"
module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
hrdf = "hrdf/6-DoF_arm.hrdf"
p = arm.ArmParams(family_name, module_names, hrdf)
a = arm.Arm(p)

# Time setup
t0 = time()
t = t0
dur = 20

# Run for 20 seconds
while (t - t0) < dur:
  a.update()
  t = time()
  # No need to set a goal since when no goal is set the arm is automatically returns to grav comp mode
    
  a.send()
