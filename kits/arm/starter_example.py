#!/usr/bin/env python3

import hebi
import os
from time import sleep
from plotting import draw_plots

# Initialize the interface for network connected modules
# Lookup class is used to find modules on the network
lookup = hebi.Lookup()
sleep(2)    # Wait for a few seconds to find modules

###############
## Arm Setup ##
###############

# Config file contains the necessary information to set up the arm, such as
# the family name, module names, path to HRDF file for the kinematics, etc.

# By default, this example uses a 6-DoF arm with X-Series modules
# You can edit the config file to match your robot's configuration
# For example, if you have a 5-DoF T-Series arm, replace the config file
# with "config/A-2580-05.cfg.yaml"

example_config_file = "config/A-2085-06.cfg.yaml"    # Relative to this file directory
example_config = hebi.config.load_config(os.path.join(os.path.dirname(os.path.realpath(__file__)), example_config_file))

# Set up arm from config
arm = hebi.arm.create_from_config(example_config, lookup)
arm.group.feedback_frequency = 200.0

###########################
## Command Home Position ##
###########################

goal = hebi.arm.Goal(arm.size)
home_position = [0.0, 2.09, 2.09, 0.0, 1.57, 0.0]
goal.add_waypoint(t=3.0, position=home_position[:arm.size])
arm.update()
arm.set_goal(goal)
arm.send()

# Start background logging
enable_logging = False
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

#######################
## Main Control Loop ##
#######################

while arm.update():

    # Send the latest loaded commands to the arm. Since the goal is already set,
    # it will send the last loaded command when arm.update() was last called
    
    arm.send()

##########################
## Logging and Plotting ##
##########################

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Add additional plotting code here
