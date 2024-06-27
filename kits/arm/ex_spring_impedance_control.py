"""
In this example we will make the end-effector behave like a virtual spring and damper.
This comprises the following demos:
- Springs along and about all axes: The end-effector springs back to the mean position and orientation.
- Cartesian springs: The end-effector springs back to the mean position but not the mean orientation.
- Torsion springs: The end-effector springs back to the mean orientation but not the mean position. Emulates a springy gimbal.
- Springy plane: The end-effector springs back to the mean orientation and z-coordinate, but can freely move in the x-y plane and spin about the z-axis.
"""
#!/usr/bin/env python3

import hebi
from enum import Enum, auto
from time import sleep
from hebi.util import create_mobile_io
from matplotlib import pyplot as plt

class Spring(Enum):
    """ Denotes the types of springs used in every demo.
    """
    ALL = auto(),
    CARTESIAN = auto(),
    TORSION = auto(),
    PLANE = auto()

# Specify the type of spring you want to use in the demo right here
# NOTE: Angle wraparound is an unresolved issue which can lead to unstable behaviour for any case involving rotational springs. 
#       Make sure that the rotational gains are high enough to prevent large angular errors. The gains provided in these examples are (mostly) well behaved.
#       Also ensure that nothing prevents the wrist's actuators from moving, and do not place your fingers between them. 
#       Interacting with the end-effector is perfectly safe.
# spring = Spring.ALL
# spring = Spring.CARTESIAN
# spring = Spring.TORSION 
spring = Spring.PLANE

# Initialize the interface for network connected modules
lookup = hebi.Lookup()
sleep(2)

# Set up arm
phone_family = "HEBIArm"
phone_name = "mobileIO"
arm_family = "HEBIArm"
hrdf_file = "hrdf/A-2085-06.hrdf"
gains_file = "gains/A-2085-06.xml"

# Set up Mobile IO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
    raise RuntimeError("Could not find Mobile IO device")
m.set_button_mode(1, 'momentary')
m.set_button_label(1, '📈')
m.set_button_mode(2, 'toggle')
m.set_button_label(2, '🌀')
m.update()

# Setup arm components
arm = hebi.arm.create([arm_family],
                      names=['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
                      lookup=lookup,
                      hrdf_file=hrdf_file)
arm.load_gains(gains_file)

impedance_controller = hebi.arm.ImpedanceController()

# Clear all position control gains for all the actuators
cmd = arm.pending_command

cmd.position_kp = 0.0
cmd.position_kd = 0.0
cmd.position_ki = 0.0

# Configure arm components
arm.add_plugin(impedance_controller)

if spring == Spring.ALL:

    impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
    impedance_controller.set_kp(100, 100, 100, 5, 5, 1)

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.gains_in_end_effector_frame = True

elif spring == Spring.CARTESIAN:

    impedance_controller.set_kd(5, 5, 5, 0.5, 0.5, 0.5)
    impedance_controller.set_kp(100, 100, 100, 0, 0, 0)

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.gains_in_end_effector_frame = False

elif spring == Spring.TORSION:

    impedance_controller.set_kd(0, 0, 0, 0, 0, 0.0)
    impedance_controller.set_kp(0, 0, 0, 5, 5, 1)

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.gains_in_end_effector_frame = True

elif spring == Spring.PLANE:

    impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
    impedance_controller.set_kp(0, 0, 100, 5, 5, 0)

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.gains_in_end_effector_frame = False

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

print("After2", cmd.position_kp, cmd.position_kd, cmd.position_ki)

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

        elif (m.get_button_diff(2) == -1):
            
            controller_on = False

        # If in impedance mode set led blue
        m.set_led_color("blue" if controller_on else "green", blocking=False)

    if not controller_on:
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
