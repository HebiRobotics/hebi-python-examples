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

The following example is for the "Floor" demo:
"""
#!/usr/bin/env python3

import hebi
from time import sleep
from hebi.util import create_mobile_io
import numpy as np
from plotting import draw_plots

# NOTE: Angle wraparound is an unresolved issue which can lead to unstable behaviour for any case involving rotational positional control. 
#       Make sure that the rotational gains are high enough to prevent large angular errors. The gains provided in these examples are (mostly) well behaved.
#       Interacting with the end-effector in these examples is perfectly safe.
#       However, ensure that nothing prevents the wrist's actuators from moving, and DO NOT place your fingers between them. 

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Set up arm
phone_family = "HEBIArm"
phone_name = "mobileIO"
example_config_file = "config/examples/ex_impedance_control_floor.cfg"

# Set up Mobile IO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
    raise RuntimeError("Could not find Mobile IO device")
m.set_button_mode(1, 'momentary')
m.set_button_label(1, '📈')
m.set_button_mode(2, 'toggle')
m.set_button_label(2, '💪')
m.update()

# Set up arm configuration
example_config = hebi.config.load_config(example_config_file)
arm = hebi.arm.create_from_config(example_config)

# Clear all position control gains for all the actuators
cmd = arm.pending_command

cmd.position_kp = 0.0
cmd.position_kd = 0.0
cmd.position_ki = 0.0

# Initialize floor demo variables
floor_level = 0.0
floor_buffer = 0.01 # 1cm

# Initialize floor demo flags
floor_command_flag = False # Indicates whether or not to command floor stiffness goals
cancel_command_flag = False # Indicates whether or not to cancel goals

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
while not m.get_button_state(1):

    # if not arm.update():
    #     print("Failed to update arm")
    #     continue
    arm.update()

    arm.send()

    if m.update(timeout_ms=0):

        # Set and unset impedance mode when button is pressed and released, respectively
        if (m.get_button_diff(2) == 1):

            controller_on = True
            arm.set_goal(goal.clear().add_waypoint(position=arm.last_feedback.position))

            # Store current height as floor level, for floor demo

            # Use forward kinematics to find end-effector pose
            ee_pose0 = np.eye(4)
            arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose0)

            # Give a little margin to floor level
            floor_level = ee_pose0[2,3] - floor_buffer

            # Update flags to indicate having left the floor
            cancel_command_flag = True

        elif (m.get_button_diff(2) == -1):
            
            controller_on = False

        # If in impedance mode set led blue
        m.set_led_color("blue" if controller_on else "green", blocking=False)

    if not controller_on:
        arm.cancel_goal()

    # Check when end-effector travels below the floor, for floor demo
    else:

        # Use forward kinematics to calculate pose of end-effector
        ee_pose_curr = np.eye(4)
        arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose_curr)

        # Snap goal to floor if end-effector is at or below floor, only when it first reaches the floor
        if ee_pose_curr[2,3] <= floor_level and floor_command_flag:

            # Snap current pose to floor
            ee_pose_floor = ee_pose_curr
            ee_pose_floor[2,3] = floor_level

            # Use inverse kinematics to calculate appropriate joint positions
            position_floor = arm.ik_target_xyz_so3(arm.last_feedback.position, ee_pose_floor[0:3,3], ee_pose_floor[0:3,0:3])

            # Set snapped pose as goal
            arm.set_goal(goal.clear().add_waypoint(t=0.1, position=position_floor))

            # Update flags to indicate being in contact with the floor
            floor_command_flag = False
            cancel_command_flag = True

        # Cancel goal if end-effector is above the floor, only when it leaves the floor
        elif ee_pose_curr[2,3] > floor_level and cancel_command_flag:

            # Cancel goal to move freely
            arm.cancel_goal()

            # Update flags to indicate having left the floor
            cancel_command_flag = False
            floor_command_flag = True

m.set_led_color("red")

if enable_logging:
    hebi_log = arm.group.stop_log()
    draw_plots(hebi_log)

    # Put more plotting code here
