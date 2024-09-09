#!/usr/bin/env python3

import hebi
from time import sleep
from hebi_util import create_mobile_io_from_config
from plotting import draw_plots

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/ex_gravity_compensation_toggle.cfg.yaml"
example_config = hebi.config.load_config(example_config_file)

# Set up arm, and mobile_io from config
arm = hebi.arm.create_from_config(example_config, lookup)
mobile_io = create_mobile_io_from_config(example_config, lookup)

# Retrieve the gravity compensation plugin
gravcomp = arm.get_plugin_by_type(hebi.arm.GravCompEffortPlugin)

arm.group.feedback_frequency = 200.0

# Start background logging
enable_logging = True
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

# Print instructions
instructions = """
  🌍 (B2) - Toggles gravity compensation on/off:
            ON  - Enable gravity compensation
            OFF - Disable gravity compensation

  📈 (B1) - Exits the demo, and plots graphs. May take a while."""

print(instructions)

#######################
## Main Control Loop ##
#######################

# while button 1 is not pressed
while not mobile_io.get_button_state(1):

    # When no goal is set, the arm automatically returns to grav-comp mode
    # Thus, the arm is in grav-comp mode awaiting further instructions,
    # even if we had an empty control loop

    # Send the latest loaded commands to the arm. If no changes are made,
    # it will send the last loaded command when arm.update() was last called
    arm.update()
    arm.send()

    if mobile_io.update(timeout_ms=0):

        # Toggle gravity compensation when button is pressed
        diff = mobile_io.get_button_diff(2)
        if diff in {-1, 1}: gravcomp.enabled = diff == 1

        # If in gravity compensation mode, set led blue
        mobile_io.set_led_color("blue" if gravcomp.enabled else "green", blocking=False)

##########################
## Logging and Plotting ##
##########################

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Insert additional plotting code here
