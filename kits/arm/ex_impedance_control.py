#!/usr/bin/env python3

import hebi
import numpy as np
import os
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api
from matplotlib import pyplot as plt

# Arm setup
phone_family = "Arm"
phone_name = "mobileIO"
arm_family = "Arm"
hrdf_file = "hrdf/A-2085-06G.hrdf"

lookup = hebi.Lookup()
sleep(2)

# Setup Mobile IO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
    raise RuntimeError("Could not find Mobile IO device")
m.set_button_mode(1, 'momentary')
m.set_button_mode(2, 'toggle')
m.update()

# Setup arm components
arm = arm_api.create([arm_family],
                     names=['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
                     lookup=lookup,
                     hrdf_file=hrdf_file)
impedance_controller = arm_api.ImpedanceController()

# Configure arm components
arm.add_plugin(impedance_controller)

# hold position only (Allow rotation around end-effector position)
impedance_controller.gains_in_end_effector_frame = True
impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
impedance_controller.set_kp(500, 500, 500, 0, 0, 0)

# Increase feedback frequency since we're calculating velocities at the
# high level for damping. Going faster can help reduce a little bit of
# jitter for fast motions, but going slower (100 Hz) also works just fine
# for most applications.
arm.group.feedback_frequency = 200.0

# Double the effort gains from their default values, to make the arm more sensitive for tracking force.
# TODO

enable_logging = True
goal = arm_api.Goal(arm.size)

# Start background logging
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

print('Commanded gravity-compensated zero force to the arm.')
print('  b2 - Toggles an impedance controller on/off:')
print('          ON  - Apply controller based on current position')
print('          OFF - Go back to gravity-compensated mode')
print('  b1 - Exits the demo.')

controller_on = False

# while button 1 is not pressed
while not m.get_button_state(1):

    if not arm.update():
        print("Failed to update arm")
        continue

    if m.update(timeout_ms=0):
        # If button 2 is pressed set to impedance mode
        controller_on = bool(m.get_button_state(2))
        # If in impedance mode set led blue
        m.set_led_color("blue" if controller_on else "green", blocking=False)

    arm.send()

    if controller_on:
        arm.set_goal(goal.clear().add_waypoint(position=arm.last_feedback.position))
    else:
        arm.cancel_goal()

m.set_led_color("red")

if enable_logging:
    hebi_log = arm.group.stop_log()

    # Plot tracking / error from the joints in the arm.
    time = []
    position = []
    velocity = []
    effort = []
    # iterate through log
    for entry in hebi_log.feedback_iterate:
        time.append(entry.transmit_time)
        position.append(entry.position)
        velocity.append(entry.velocity)
        effort.append(entry.effort)

    # Offline Visualization
    # Plot the logged position feedback
    plt.figure(101)
    plt.plot(time, position)
    plt.title('Position')
    plt.xlabel('time (sec)')
    plt.ylabel('position (rad)')
    plt.grid(True)

    # Plot the logged velocity feedback
    plt.figure(102)
    plt.plot(time, velocity)
    plt.title('Velocity')
    plt.xlabel('time (sec)')
    plt.ylabel('velocity (rad/sec)')
    plt.grid(True)

    # Plot the logged effort feedback
    plt.figure(103)
    plt.plot(time, effort)
    plt.title('Effort')
    plt.xlabel('time (sec)')
    plt.ylabel('effort (N*m)')
    plt.grid(True)

    plt.show()

    # Put more plotting code here
