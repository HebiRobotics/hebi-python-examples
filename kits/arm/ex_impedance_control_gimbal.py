"""
In this example we will implement various hybrid motion-force controllers using the impedance control plugin, which can be used for a wide variety of 
applications.
Impedance control is BEST SUITED for enabling free, rigid and springy behaviour, along/about each different axis.
While this is perfectly useful for:
- Having a selectively compliant end-effector,
- Switching between fixed and free behaviour to simulate (mostly) rigid constraints, and
- Allowing human intervention for automated operations by separating controls across different axes,
any applications involving more salient control of the forces (as more complex functions with flexible inputs) should use our force control plugin. See ex_force_control_demoname.py.

This comprises the following demos:
- Fixed: A task-space pose controller implemented entirely using force control via the (PD) impedance controller.
- Cartesian: Locks onto a particular end-effector position while having some compliant orientation.
- Gimbal: A gimbal that locks a specific end-effector orientation, while keeping the rest of the arm compliant.
- Floor: The end-effector is free to move but can't travel below a virtual floor. To further simulate sliding on the floor, see force_control example.
- Damping: The end-effector behaves as 3-different damped systems (overdamped, critically damped, and underdamped), at 3 different heights.

The following example is for the "Gimbal" demo:
"""
#!/usr/bin/env python3

import hebi
from time import sleep
from hebi_util import create_mobile_io_from_config
import numpy as np
from plotting import draw_plots

# NOTE: Angle wraparound is an unresolved issue which can lead to unstable behaviour for any case involving rotational positional control. 
#       Make sure that the rotational gains are high enough to prevent large angular errors. The gains provided in these examples are (mostly) well behaved.
#       Interacting with the end-effector in these examples is perfectly safe.
#       However, ensure that nothing prevents the wrist's actuators from moving, and DO NOT place your fingers between them. 

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Config file
example_config_file = "config/ex_impedance_control_gimbal.cfg.yaml"
example_config = hebi.config.load_config(example_config_file)

# Set up arm, and mobile_io from config
arm = hebi.arm.create_from_config(example_config, lookup)
mobile_io = create_mobile_io_from_config(example_config, lookup)

# Clear all position control gains for all the actuators
cmd = arm.pending_command

cmd.position_kp = 0.0
cmd.position_kd = 0.0
cmd.position_ki = 0.0

# Increase feedback frequency since we're calculating velocities at the
# high level for damping. Going faster can help reduce a little bit of
# jitter for fast motions, but going slower (100 Hz) also works just fine
# for most applications.
arm.group.feedback_frequency = 200.0

# Double the effort gains from their default values, to make the arm more sensitive for tracking force.
# TODO

enable_logging = True
goal = hebi.arm.Goal(arm.size)

# Start background logging
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)
    
# Print instructions
instructions = """Commanded gravity-compensated zero force to the arm.

  🛑 (B2) - Toggles an impedance controller on/off:
            ON  - Apply controller based on current orientation
            OFF - Go back to gravity-compensated mode

  📈 (B1) - Exits the demo, and plots graphs. May take a while."""

print(instructions)

#######################
## Main Control Loop ##
#######################

controller_on = False

# while button 1 is not pressed
while not mobile_io.get_button_state(1):

    # if not arm.update():
    #     print("Failed to update arm")
    #     continue
    arm.update()

    arm.send()

    if mobile_io.update(timeout_ms=0):

        # Set and unset impedance mode when button is pressed and released, respectively
        if (mobile_io.get_button_diff(2) == 1):

            controller_on = True
            arm.set_goal(goal.clear().add_waypoint(position=arm.last_feedback.position))

        elif (mobile_io.get_button_diff(2) == -1):
            
            controller_on = False

        # If in impedance mode set led blue
        mobile_io.set_led_color("blue" if controller_on else "green", blocking=False)

    if not controller_on:
        arm.cancel_goal()

mobile_io.set_led_color("red")

##########################
## Logging and Plotting ##
##########################

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Put more plotting code here
