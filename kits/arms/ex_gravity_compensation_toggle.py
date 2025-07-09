#!/usr/bin/env python3

import hebi
import os
from time import sleep
from hebi_util import create_mobile_io_from_config
from plotting import draw_plots

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/ex_gravity_compensation_toggle.cfg.yaml"  # Relative to this file directory
example_config = hebi.config.load_config(os.path.join(os.path.dirname(os.path.realpath(__file__)), example_config_file))

# Set up arm, and mobile_io from config
arm = hebi.arm.create_from_config(example_config, lookup)
while not (mobile_io := create_mobile_io_from_config(example_config, lookup)):
    print('Looking for mobileIO device')
    sleep(1)

# Retrieve the gravity compensation plugin
gravcomp = arm.get_plugin_by_type(hebi.arm.GravCompEffortPlugin)
if not gravcomp:
    print('Config does not have grav comp plugin, use a different config for this demo')
    exit()

arm.group.feedback_frequency = 200.0

# Start background logging
enable_logging = False
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

# Print instructions
instructions = """GRAVITY COMPENSATION EXAMPLE

(On/Off) - Toggles gravity compensation on/off:
        ON  - Enable gravity compensation
        OFF - Disable gravity compensation

(Quit) - Exits the demo, and plots graphs if logging is enabled."""

print(instructions)
mobile_io.add_text(instructions)

#######################
## Main Control Loop ##
#######################

switch_btn = 1
quit_btn = 2

# while quit is not pressed
while not mobile_io.get_button_state(quit_btn):
    arm.update()

    # When no goal is set, the arm automatically returns to grav-comp mode
    # Thus, the arm is in grav-comp mode awaiting further instructions,
    # even if we had an empty control loop

    if mobile_io.update(timeout_ms=0):

        # Toggle gravity compensation when button is pressed
        diff = mobile_io.get_button_diff(switch_btn)
        if diff in {-1, 1}:
            gravcomp.enabled = diff == 1

        # If in gravity compensation mode, set led blue
        mobile_io.set_led_color("blue" if gravcomp.enabled else "green", blocking=False)
    
    # Send the latest loaded commands to the arm. If no changes are made,
    # it will send the last loaded command when arm.update() was last called
    arm.send()

mobile_io.set_led_color("red")

##########################
## Logging and Plotting ##
##########################

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Add additional plotting code here
