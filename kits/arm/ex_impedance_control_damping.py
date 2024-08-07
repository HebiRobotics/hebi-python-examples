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

The following example is for the "Damping" demo:
"""
#!/usr/bin/env python3

import hebi
from time import sleep
from demo_util import create_demo_from_config
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
example_config_file = "config/examples/ex_impedance_control_damping.cfg.yaml"

# Set up arm, and mobile_io from config
example_config = hebi.config.load_config(example_config_file)
arm, mobile_io, _ = create_demo_from_config(lookup, example_config)
mobile_io.set_button_mode(2, 'toggle')
mobile_io.update()

# Retrieve the impedance control plugin to be altered later
impedance_controller = arm.get_plugin_by_type(hebi.arm.ImpedanceController)

# Clear all position control gains for all the actuators
cmd = arm.pending_command

cmd.position_kp = 0.0
cmd.position_kd = 0.0
cmd.position_ki = 0.0

# Meters above the base for overdamped, critically damped, and underdamped cases respectively
lower_limits = example_config.user_data['lower_limits'] 
# State variable for current mode: 0 for overdamped, 1 for crtically damped, 2 for underdamped, -1 for free
mode = -1
prevmode = -1

# Arrange different gains in an ordered list
damping_kp = [example_config.user_data['overdamped']['kp'], 
              example_config.user_data['critically_damped']['kp'], 
              example_config.user_data['underdamped']['kp']]
damping_kd = [example_config.user_data['overdamped']['kd'], 
              example_config.user_data['critically_damped']['kd'], 
              example_config.user_data['underdamped']['kd']]

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

print('Commanded gravity-compensated zero force to the arm.')
print('  💪 (B2) - Toggles an impedance controller on/off:')
print('            ON  - Apply controller based on current position')
print('            OFF - Go back to gravity-compensated mode')
print('  📈 (B1) - Exits the demo, and plots graphs. May take a while.')

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
        mode = -1 # Free
        prevmode = -1

    # Check when end-effector travels below the floor, for floor demo
    else:

        # Use forward kinematics to calculate pose of end-effector
        ee_pose_curr = np.eye(4)
        arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose_curr)

        # Assign mode based on current position
        for i in range(len(lower_limits)):

            if ee_pose_curr[2,3] > lower_limits[i]:

                mode = i
        
        # Change gains only upon mode switches
        if not mode == prevmode and mode >= 0:

            impedance_controller.set_kp(*damping_kp[mode])
            impedance_controller.set_kd(*damping_kd[mode])

            print(f"Mode: {mode}")

        prevmode = mode

mobile_io.set_led_color("red")

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Put more plotting code here
