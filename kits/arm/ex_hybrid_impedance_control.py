"""
In this example we will implement various hybrid motion-force controllers using the impedance control plugin, which can be used for a wide variety of 
applications.
This is different from the "spring" example set as here the intent is not to create a virtual analog of a physical system (such as a spring and damper), 
but rather to implement robust hybrid motion-force controllers.

This comprises the following demos:
- Position control: A task-space position controller implemented entirely using force control via the impedance controller.
- Gimbal: A gimbal that locks a specific end-effector orientation, while keeping the rest of the arm compliant.
- Helical screw: An example where "screwing" or "unscrewing" the end-effector makes it translate forwards or backwards in along whatever pose it is locked into.
- Floor: The end-effector is free to move but can't travel below a virtual floor.
- Pipe: The end effector can only move along the surface of a virtual cylinder.
"""
#!/usr/bin/env python3

import hebi
from enum import Enum, auto
from time import sleep
from hebi.util import create_mobile_io
from matplotlib import pyplot as plt
import numpy as np

class State(Enum):
    """ Used to denote each specific demo.
    """
    POSITION = auto(),
    GIMBAL = auto(),
    SCREW = auto(),
    FLOOR = auto(),
    PIPE = auto()

# Specify the type of spring you want to use in the demo right here
# NOTE: Angle wraparound is an unresolved issue which can lead to unstable behaviour for any case involving rotational positional control. 
#       Make sure that the rotational gains are high enough to prevent large angular errors. The gains provided in these examples are (mostly) well behaved.
#       Interacting with the end-effector in these examples is perfectly safe.
#       However, ensure that nothing prevents the wrist's actuators from moving, and DO NOT place your fingers between them. 
# state = State.POSITION # ✅
# state = State.GIMBAL # ✅
# state = State.SCREW # ❌
state = State.FLOOR # ✅
# state = State.PIPE # ❌

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
m.set_button_label(2, '💪')
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

if state == State.POSITION:

    impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
    impedance_controller.set_kp(200, 200, 200, 5, 5, 1)

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.gains_in_end_effector_frame = True

elif state == State.GIMBAL:

    impedance_controller.set_kd(0, 0, 0, 0, 0, 0)
    impedance_controller.set_kp(0, 0, 0, 8, 8, 1)

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.gains_in_end_effector_frame = True

# TODO: Angle wraparound needs to be fixed in the API
# elif state == State.SCREW:

#     impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
#     impedance_controller.set_kp(200, 200, 200, 5, 5, 0)

#     # Dictate impedance controller gains in SE(3) based on the state
#     impedance_controller.gains_in_end_effector_frame = True

#     screw_angle0 = 0.0
#     screw_angle = 0.0
#     screw_pitch = 2.0

elif state == State.FLOOR:

    # impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
    # impedance_controller.set_kp(100, 100, 100, 5, 5, 1)

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.gains_in_end_effector_frame = True

    # Initialize floor demo variables
    floor_level = 0.0
    floor_buffer = 0.01 # 1cm

    # Initialize floor demo flags
    floor_command_flag = False # indicates whether or not to command floor stiffness goals
    cancel_command_flag = False # Indicates whether or not to cancel goals

# TODO: Need more time
# elif state == State.PIPE:

    # impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
    # impedance_controller.set_kp(100, 100, 100, 5, 5, 1)

    # # Dictate impedance controller gains in SE(3) based on the state
    # impedance_controller.gains_in_end_effector_frame = True

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

            # Store original screw angle, for screw demo
            if state == State.SCREW:

                screw_angle0 = arm.last_feedback.position[5]

            # Store current height as floor level, for floor demo
            elif state == State.FLOOR:

                # Use forward kinematics to find end-effector pose
                ee_pos0 = np.eye(4)
                arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pos0)

                # Give a little margin to floor level
                floor_level = ee_pos0[2,3] - floor_buffer

                # Update flags to indicate having left the floor
                cancel_command_flag = True

        elif (m.get_button_diff(2) == -1):
            
            controller_on = False

        # If in impedance mode set led blue
        m.set_led_color("blue" if controller_on else "green", blocking=False)

    if not controller_on:
        arm.cancel_goal()

    # Command new positions based on screw angle, for screw demo
    elif controller_on and state == State.SCREW:

        screw_angle = arm.last_feedback.position[5] - screw_angle0
        # print(screw_angle)
        # arm.set_goal(goal.clear().add_waypoint(position=arm.last_feedback.position))

    # Check when end-effeector travels below the floor, for floor demo
    elif controller_on and state == State.FLOOR:

        # Use forward kinematics to calculate pose of end-effector
        ee_pose_curr = np.eye(4)
        arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose_curr)

        # Snap goal to floor if end-effector is at or below floor, only when it first reaches the floor
        if ee_pose_curr[2,3] <= floor_level and floor_command_flag:

            # Set surface stiffness of floor
            impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
            impedance_controller.set_kp(500, 500, 500, 5, 5, 1)

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

            # Set stiffness in air
            impedance_controller.set_kd(0, 0, 0, 0, 0, 0)
            impedance_controller.set_kp(0, 0, 0, 5, 5, 1)

            # Cancel goal to move freely
            arm.cancel_goal()

            # Update flags to indicate having left the floor
            cancel_command_flag = False
            floor_command_flag = True

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
