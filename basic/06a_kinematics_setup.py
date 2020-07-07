#!/usr/bin/env python3

import hebi
from math import pi

# Method 1: Load the kinematics from an HRDF file
try:
  model_from_hrdf = hebi.robot_model.import_from_hrdf("hrdf/A-2085-03.hrdf")
except:
  print("Could not load HRDF.")
  exit(1)

print('Robot model loaded from HRDF has {0} degrees of freedom.'.format(model_from_hrdf.dof_count))

model_from_code = hebi.robot_model.RobotModel()
model_from_code.add_actuator('X5-9')
model_from_code.add_bracket('X5-HeavyBracket', 'left-outside')
model_from_code.add_actuator('X5-9')
model_from_code.add_link('X5', 0.325, pi)
model_from_code.add_actuator('X5-4')
model_from_code.add_link('X5', 0.325, 0.0)

# Method 2: Create a simple kinematic description of the arm in code
print('Robot model generated in code has {0} degrees of freedom.'.format(model_from_code.dof_count))
