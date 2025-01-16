#!/usr/bin/env python3

import hebi
import os
from time import sleep
from plotting import draw_plots

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/ex_gravity_compensation.cfg.yaml"   # Relative to this file directory
example_config = hebi.config.load_config(os.path.join(os.path.dirname(os.path.realpath(__file__)), example_config_file))

# Set up arm from config
arm = hebi.arm.create_from_config(example_config, lookup)

arm.group.feedback_frequency = 200.0

# Start background logging
enable_logging = False
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

#######################
## Main Control Loop ##
#######################

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

    # Add additional plotting code here
