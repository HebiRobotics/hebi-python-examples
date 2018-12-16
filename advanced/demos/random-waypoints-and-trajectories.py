import hebi
import numpy
from time import sleep
import random


def get_grav_comp_efforts(robot_model, positions, gravityVec):
    # Normalize gravity vector (to 1g, or 9.8 m/s^2)
    normed_gravity = gravityVec / numpy.linalg.norm(gravityVec) * 9.81

    jacobians = robot_model.get_jacobians('CoM', positions)
    # Get torque for each module
    # comp_torque = J' * wrench_vector
    # (for each frame, sum this quantity)
    comp_torque = numpy.zeros((robot_model.dof_count, 1))

    # Wrench vector
    wrench_vec = numpy.zeros(6)  # For a single frame; this is (Fx/y/z, tau x/y/z)
    num_frames = robot_model.get_frame_count('CoM')

    for i in range(num_frames):
        # Add the torques for each joint to support the mass at this frame
        wrench_vec[0:3] = normed_gravity * robot_model.masses[i]
        comp_torque += jacobians[i].transpose() * numpy.reshape(wrench_vec, (6, 1))

    return numpy.squeeze(comp_torque)


def setup():
  lookup = hebi.Lookup()
  group = lookup.get_group_from_names(['HEBI'], ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3'])

  if group is None:
    print('Group not found: Did you forget to set the module family and names above?')
    exit(1)

  group.feedback_frequency = 100
  group.command_lifetime = 100

  model = hebi.robot_model.RobotModel()
  model.add_actuator('X8-9')
  model.add_bracket('X5-HeavyBracket','Right-Outside')
  model.add_actuator('X8-9')
  model.add_link('X5', 0.325, numpy.pi)
  model.add_actuator('X5-9')
  model.add_link('X5', 0.325, numpy.pi)
  model.add_actuator('X5-9')
  model.add_bracket('X5-LightBracket','Right')
  model.add_actuator('X5-4')
  model.add_bracket('X5-LightBracket','Right')
  model.add_actuator('X5-4')

  payload_mass = 0.6
  model.add_rigid_body([0,0,0], [1,1,1,0,0,0], payload_mass, numpy.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]), True)
  return [group, model]


# Simple random number generator within given range
def r(low, high):
  return random.random() * (high - low) + low
  

def generate_waypoints(model, current_pos):
  return [r(-.4, .4), r(0.25, 1.2), r(0.75, 2.0), r(-1.5, 1.5), r(-1.5, 1.5), r(-0.75, 0.75)]
"""
  tip_down = hebi.robot_model.endeffector_tipaxis_objective([0.0, 0.0, -1.0])

  new_target_xyz = [random.random() * 0.2 + 0.2, random.random() * 0.4 - 0.2, random.random() * 0.25]
  print new_target_xyz
  objective = hebi.robot_model.endeffector_position_objective(new_target_xyz)

  ik = model.solve_inverse_kinematics(current_pos, objective)
  print ik
  return ik"""


def get_fbk(group):
  fbk = group.get_next_feedback()
  if fbk is None:
    print('Could not get feedback')
    raise RuntimeError('Could not get feedback')
  return fbk


def play_trajectory(group, model, trajectory, fraction = 1.0):
  t = 0.0
  period = 0.01
  cmd = hebi.GroupCommand(6)
  spring_offset = numpy.zeros((1, 6))
  spring_offset[0, 1] = -9

  duration = trajectory.duration
  while (t < duration * fraction):
    fbk = get_fbk(group)
    pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
    cmd.position = pos_cmd
    cmd.velocity = vel_cmd
    cmd.effort = numpy.matrix(get_grav_comp_efforts(model, fbk.position, [0.0, 0.0, 1.0]) + spring_offset).A1
    group.send_command(cmd)
    t = t + period # TODO: do this better!

def run():
  [group, model] = setup()

  start_pt = [0, 1, 0.4, 0, 0, 0]

  pos = get_fbk(group).position
#  waypoint1 = generate_waypoints(model, pos)
  waypoint1 = start_pt
  waypoint2 = generate_waypoints(model, waypoint1)

  # Pause for 2.5 seconds at start pos
  vel = numpy.zeros((6, 4))
  acc = numpy.zeros((6, 4))
  time = numpy.linspace(0.0, 7.5, 4)
  wp = numpy.matrix([pos, waypoint1, waypoint1, waypoint2])
  trajectory = hebi.trajectory.create_trajectory(time, wp.T, vel, acc)

  play_trajectory(group, model, trajectory, 2.0 / 3.0) # The 2/3 means no stop to final waypoint

  # Reset to 3-long time vector
  vel = numpy.zeros((6, 3))
  acc = numpy.zeros((6, 3))
  vel[:,1] = acc[:,1] = numpy.nan
  time = numpy.linspace(0.0, 5.0, 3)

  # Start Logging
  group.start_log()

  numMoves = 50
  i = 0
  while i < numMoves:
    # Update waypoints and regenerate trajectory
    next_waypoint = generate_waypoints(model, waypoint2)
    wp = numpy.matrix([waypoint1, waypoint2, next_waypoint])
    waypoint1 = waypoint2
    waypoint2 = next_waypoint
   
    duration = trajectory.duration 
    pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(duration - 2.5)
    vel[:,0] = vel_cmd
    acc[:,0] = acc_cmd

    trajectory = hebi.trajectory.create_trajectory(time, wp.T, vel, acc)
 
    # Play through first half
    play_trajectory(group, model, trajectory, 0.5)

    i = i + 1


  # complete to starting point
  wp = numpy.matrix([waypoint1, waypoint2, start_pt])
  trajectory = hebi.trajectory.create_trajectory(time, wp.T, vel, acc)
  play_trajectory(group, model, trajectory, 1.0)

  t = 0
  period = 0.01
  fbk = get_fbk(group)
  while t < 20:
    cmd = hebi.GroupCommand(6)
    spring_offset = numpy.zeros((1, 6))
    spring_offset[0, 1] = -9
    cmd.position = start_pt
    cmd.effort = numpy.matrix(get_grav_comp_efforts(model, fbk.position, [0.0, 0.0, 1.0]) + spring_offset).A1
    group.send_command(cmd)
    sleep(period)
    t = t + period

  group.stop_log()

  return

run()
