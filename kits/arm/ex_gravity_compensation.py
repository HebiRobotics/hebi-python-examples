#!/usr/bin/env python3

import hebi
from time import sleep
from hebi.util import create_mobile_io_from_config
from plotting import draw_plots

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Arm setup
example_config_file = "config/examples/ex_gravity_compensation.cfg.yaml"

# Create Arm object
example_config = hebi.config.load_config(example_config_file)
arm = hebi.arm.create_from_config(example_config)

# Retrieve the gravity compensation plugin
gravcomp = arm.get_plugin_by_type(hebi.arm.GravCompEffortPlugin)

# Set up Mobile IO from config
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io_from_config(lookup, example_config)
m.set_button_mode(2, 'toggle')
m.update()

arm.group.feedback_frequency = 200.0

# Start background logging
enable_logging = True
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

#######################
## Main Control Loop ##
#######################

print('Commanding gravity-compensated zero torques to the arm.')
# while button 1 is not pressed
while not m.get_button_state(1):

    # When no goal is set, the arm automatically returns to grav-comp mode
    # Thus, the arm is in grav-comp mode awaiting further instructions,
    # even if we had an empty control loop

    # Send the latest loaded commands to the arm. If no changes are made,
    # it will send the last loaded command when arm.update() was last called
    arm.update()
    arm.send()

    if m.update(timeout_ms=0):

        # Toggle gravity compensation when button is pressed
        diff = m.get_button_diff(2)
        if diff in {-1, 1}: gravcomp.enabled = diff == 1

        # If in gravity compensation mode, set led blue
        m.set_led_color("blue" if gravcomp.enabled else "green", blocking=False)

##########################
## Logging and Plotting ##
##########################

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Insert additional plotting code here
