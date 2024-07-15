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
    """ Used to specify the behaviour in a specific demo.
    """
    FIXED = auto(),
    WEIGHT = auto(),
    BALL_ON_FLOOR = auto(),
    ROPE = auto(),
    POP = auto(),
    GAME = auto(),

# Specify the type of spring you want to use in the demo right here
# NOTE: Angle wraparound is an unresolved issue which can lead to unstable behaviour for any case involving rotational positional control. 
#       Make sure that the rotational gains are high enough to prevent large angular errors. The gains provided in these examples are (mostly) well behaved.
#       Interacting with the end-effector in these examples is perfectly safe.
#       However, ensure that nothing prevents the wrist's actuators from moving, and DO NOT place your fingers between them. 
# state = State.FIXED # ❌
# state = State.BALL_ON_FLOOR # ❌
state = State.ROPE # ✅
# state = State.GAME # ❌
# state = State.WEIGHT # ✅ WARNING: Activate while end-effector is resting on a surface to avoid harm.

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

if state == State.FIXED:

    # Dictate impedance controller gains in SE(3) based on the state
    impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
    impedance_controller.set_kp(200, 200, 200, 5, 5, 1)

    # Keep in end-effector frame since it is more intuitive to define rotational stiffness
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

elif state == State.BALL_ON_FLOOR:

    # No need to specify gains now since they will keep switching later

    # Keep in end-effector frame since it is more intuitive to define rotational stiffness
    impedance_controller.gains_in_end_effector_frame = True

    # Initialize floor demo variables
    floor_level = 0.0
    floor_buffer = 0.01 # 1cm

    # Initialize floor demo flags
    floor_command_flag = False # indicates whether or not to command floor stiffness goals
    cancel_command_flag = False # Indicates whether or not to cancel goals

elif state == State.ROPE:

    # No need to specify gains now since they will keep switching later
    impedance_controller.set_kd(0, 0, 0, 0, 0, 0)
    impedance_controller.set_kp(0, 0, 0, 0, 0, 0)
    
    # Keep in end-effector frame since it is more intuitive to define rotational stiffness
    impedance_controller.gains_in_end_effector_frame = False

    # Initialize floor demo variables
    anchor_point = np.zeros(3)
    rope_length = 0.2 # meters
    rope_stiffness = 500 

elif state == State.GAME:

    # No need to specify gains now since they will keep switching later
    impedance_controller.set_kd(0, 0, 0, 0, 0, 0)
    impedance_controller.set_kp(0, 0, 0, 0, 0, 0)

    hit_radius = 0.3 # Easy: 0.3m, Medium: 0.25m, Hard: 0.2m
    target_hit_flag = False 
    generate_target_flag = True
    reveal = True # Set as true to reveal each target

    lower_bounds = np.array([-0.5, -0.5, 0])
    upper_bounds = np.array([0.5, 0.5, 0.5])

    target = np.zeros(3)
    max_repulsion = 5
    score = 0

elif state == State.WEIGHT:

    impedance_controller.set_kd(0, 0, 0, 0, 0, 0)
    impedance_controller.set_kp(0, 0, 0, 0, 0, 0)

    # Keep in end-effector frame since it is more intuitive to define rotational stiffness
    impedance_controller.gains_in_end_effector_frame = False

    zone_lower_limits = [0.0, 0.2, 0.4] # meters above the base. Keep in ascending order.
    zone_weights = [10, 5, 0] # weight in each zone, from the bottom to the top

    zone = -1 # 0, 1, 2... Initialized as -1. Going from the bottom to the top.

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

    # print("AAAAA")
    # print(np.round(arm.pending_command.effort, 2))

    # arm.send()

    if m.update(timeout_ms=0):

        # Set and unset impedance mode when button is pressed and released, respectively
        if (m.get_button_diff(2) == 1):

            controller_on = True
            arm.set_goal(goal.clear().add_waypoint(position=arm.last_feedback.position))

            # Store current height as floor level, for floor demo
            if state == State.BALL_ON_FLOOR:

                # Use forward kinematics to find end-effector pose
                ee_pose0 = np.eye(4)
                arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose0)

                # Give a little margin to floor level
                floor_level = ee_pose0[2,3] - floor_buffer

                # Update flags to indicate having left the floor
                cancel_command_flag = True

            # Store current position as rope anchor point, for rope demo
            elif state == State.ROPE:

                # Use forward kinematics to find end-effector pose
                ee_pose0 = np.eye(4)
                arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose0)

                anchor_point = ee_pose0[0:3,3]

            # elif state == State.WEIGHT:

            #     # Use forward kinematics to find end-effector pose
            #     ee_pose0 = np.eye(4)
            #     arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose0)

            #     for i in range(len(zone_lower_limits)):

            #         if ee_pose0[2,3] > zone_lower_limits[i]:

            #             zone = i

        elif (m.get_button_diff(2) == -1):
            
            controller_on = False

        # If in impedance mode set led blue
        m.set_led_color("blue" if controller_on else "green", blocking=False)

    if not controller_on:
        arm.cancel_goal()

    # Check when end-effector travels below the floor, for floor demo
    if controller_on and state == State.BALL_ON_FLOOR:

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

    # Check when end-effector pulls the rop taut, for rope demo
    elif controller_on and state == State.ROPE:

        # Use forward kinematics to calculate pose of end-effector
        ee_pose_curr = np.eye(4)
        arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose_curr)

        # print("ROPPPPEE")
        # print(np.linalg.norm(ee_pose_curr[0:3, 2] - anchor_point))
        # print(ee_pose_curr[0:3, 3], anchor_point)

        # Snap goal to rope edge if end-effector pulls the rope taut, only when it first reaches the edge
        if np.linalg.norm(ee_pose_curr[0:3, 3] - anchor_point) >= rope_length:

            print("PULLED")

            # Set stiffness of taut rope
            # impedance_controller.set_kd(5, 5, 5, 0, 0, 0)
            # impedance_controller.set_kp(500, 500, 500, 5, 5, 1)

            # Snap current pose to floor
            # ee_pose_rope = ee_pose_curr
            # ee_pose_rope[0:3,3] = anchor_point + rope_length * (ee_pose_curr[0:3, 3] - anchor_point) / np.linalg.norm(ee_pose_curr[0:3, 3] - anchor_point)

            # # Use inverse kinematics to calculate appropriate joint positions
            # position_rope = arm.ik_target_xyz_so3(arm.last_feedback.position, ee_pose_rope[0:3,3], ee_pose_rope[0:3,0:3])

            displacement = ee_pose_curr[0:3, 3] - anchor_point

            desired_wrench = np.zeros(6)
            desired_wrench[0:3] = -rope_stiffness * displacement * ( 1 - (rope_length) / np.linalg.norm(displacement))

            # print(desired_wrench[0:3])
            # print(displacement, rope_stiffness, ( 1 - rope_length / np.linalg.norm(displacement)))
            print(rope_length, np.linalg.norm(displacement))

            jacobian_end_effector = np.zeros([6,6])

            arm.robot_model.get_jacobian_end_effector(arm.last_feedback.position, jacobian_end_effector)

            extra_effort = jacobian_end_effector.T @ desired_wrench

            arm_cmd = arm.pending_command
            cmd_eff = arm_cmd.effort
            cmd_eff += extra_effort

            arm_cmd.effort = cmd_eff

            arm.pending_command.effort += extra_effort

        # elif np.linalg.norm(ee_pose_curr[0:3, 3] - anchor_point) < rope_length:

    elif controller_on and state == State.WEIGHT:

        ee_pose_curr = np.eye(4)
        arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose_curr)

        for i in range(len(zone_lower_limits)):

            if ee_pose_curr[2,3] > zone_lower_limits[i]:

                zone = i
        
        desired_wrench = np.zeros(6)
        desired_wrench[2] = -zone_weights[zone]

        jacobian_end_effector = np.zeros([6,6])

        arm.robot_model.get_jacobian_end_effector(arm.last_feedback.position, jacobian_end_effector)

        extra_effort = jacobian_end_effector.T @ desired_wrench

        arm_cmd = arm.pending_command
        cmd_eff = arm_cmd.effort
        cmd_eff += extra_effort

        arm_cmd.effort = cmd_eff

        arm.pending_command.effort += extra_effort

    elif controller_on and state == State.GAME:

        if generate_target_flag:

            target = np.random.uniform(lower_bounds, upper_bounds)
            generate_target_flag = False

            if reveal:

                print(f"Target at X: {np.round(target[0], 2)}, Y: {np.round(target[1], 2)}, Z: {np.round(target[2], 2)}")

        ee_pose_curr = np.eye(4)
        arm.robot_model.get_end_effector(arm.last_feedback.position, ee_pose_curr)

        displacement = ee_pose_curr[0:3, 3] - target
        desired_wrench = np.zeros(6)
        desired_wrench[0:3] = max_repulsion * np.exp(-np.linalg.norm(displacement)) * displacement / np.linalg.norm(displacement)

        # print(desired_wrench[0:3])
        # print(target)

        jacobian_end_effector = np.zeros([6,6])

        arm.robot_model.get_jacobian_end_effector(arm.last_feedback.position, jacobian_end_effector)

        extra_effort = jacobian_end_effector.T @ desired_wrench

        arm_cmd = arm.pending_command
        cmd_eff = arm_cmd.effort
        cmd_eff += extra_effort

        arm_cmd.effort = cmd_eff

        arm.pending_command.effort += extra_effort

        if np.linalg.norm(displacement) < hit_radius:

            score += 1
            print(f"REACHED! SCORE: {score}")
            generate_target_flag = True

        # print(np.linalg.norm(displacement))

    # elif controller_on and state == State.BOOK:
            
    #     desired_wrench = np.array([0, 0, 0.0, 0, 0, 0])

    #     jacobian_end_effector = np.zeros([6,6])

    #     arm.robot_model.get_jacobian_end_effector(arm.last_feedback.position, jacobian_end_effector)

    #     extra_effort = jacobian_end_effector.T @ desired_wrench

    #     cmd_eff += extra_effort

    #     arm_cmd.effort = cmd_eff

    #     arm.pending_command.effort += extra_effort

    #     # arm.send()

    #     # print("BBBB")

    #     # print(np.round(extra_effort, 2))
    #     # print(np.round(arm.pending_command.effort, 2))

    arm.send()

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
