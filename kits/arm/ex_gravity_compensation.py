#!/usr/bin/env python3

import hebi
from plotting import draw_plots

# Arm setup
arm_family = "Arm"
module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
hrdf_file = "hrdf/A-2085-06.hrdf"
gains_file = "gains/A-2085-06.xml"


# Create Arm object
arm = hebi.arm.create([arm_family],
                      names=module_names,
                      hrdf_file=hrdf_file)
arm.load_gains(gains_file)


# Start background logging
enable_logging = True
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

#######################
## Main Control Loop ##
#######################

print('Commanding gravity-compensated zero torques to the arm.')
while arm.update():
    # When no goal is set, the arm automatically returns to grav-comp mode
    # Thus, when we have an empty control loop, the arm is in grav-comp
    # awaiting further instructions

    # Send the latest loaded commands to the arm. If no changes are made,
    # it will send the last loaded command when arm.update() was last called
    arm.send()

##########################
## Logging and Plotting ##
##########################

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Insert additional plotting code here
