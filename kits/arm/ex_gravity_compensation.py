#!/usr/bin/env python3

import hebi
from matplotlib import pyplot as plt

# Set up to find actuators on the network
lookup = hebi.Lookup()
sleep(2)

# Arm setup
arm_family = "Arm"
module_names = ['J1_base', 'J2A_shoulder1', 'J3_shoulder2', 'J4_elbow1', 'J5_elbow2', 'J6_wrist1', 'J7_wrist2']
hrdf_file = "hrdf/A-2303-01.hrdf"
gains_file = "gains/A-2303-01.xml"

root_dir = os.path.abspath(os.path.dirname(__file__))
hrdf_file = os.path.join(root_dir, hrdf_file)
gains_file = os.path.join(root_dir, gains_file)

# Create Arm object
arm = hebi.arm.create([arm_family],
                      names=module_names,
                      hrdf_file=hrdf_file)

alt_shoulder_group = lookup.get_group_from_names(arm_family, ['J2B_shoulder1'])
while not alt_shoulder_group:
    print(f"Looking for shoulder module {arm_family} / J2B_shoulder1 ...")
    sleep(1)
    alt_shoulder_group = lookup.get_group_from_names(arm_family, ['J2B_shoulder1'])

double_shoulder = hebi.arm.DoubledJointMirror(1, alt_shoulder_group)
arm.add_plugin(double_shoulder)

arm.load_gains(gains_file)


# Start background logging
enable_logging = True
if enable_logging:
    arm.group.start_log('dir', 'logs', mkdirs=True)

#######################
## Main Control Loop ##
#######################

print('Commanding gravity-compensated zero torques to the arm.')
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
    # Insert additional plotting code here
