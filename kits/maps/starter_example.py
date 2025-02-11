#!/usr/bin/env python3

import hebi
import os
from time import sleep
from plotting import draw_plots
import numpy as np

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

example_config_file = "config/mapsArm-7DoF.cfg.yaml"    # Relative to this file directory
example_config = hebi.config.load_config(os.path.join(os.path.dirname(os.path.realpath(__file__)), example_config_file))

# Set up arm from config
arm = hebi.arm.create_from_config(example_config, lookup)
arm.group.feedback_frequency = 200.0

######################
## Update Arm State ##
######################

arm.update()
arm.send()

# Fill in feedback
group_feedback = hebi.GroupFeedback(arm.group.size)
group_feedback = arm.group.get_next_feedback(reuse_fbk=group_feedback)

# Retrieve positions:
positions = group_feedback.position

# Create Variables
xyz = np.empty(3)
rot = np.empty((3, 3))

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
    # These are the current joint angles/positions, perhaps from a GroupFeedback object:
    positions = group_feedback.position

    arm.FK(positions, xyz_out=xyz, orientation_out=rot)

    print('output xyz: ', xyz)

    arm.send()

##########################
## Logging and Plotting ##
##########################

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Add additional plotting code here
