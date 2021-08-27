#!/usr/bin/env python3

import hebi
from hebi.trajectory import create_trajectory
from math import fmod, radians
from time import sleep, time


def smooth_move_to(group, cmd, end, duration=1.0):
    group.send_feedback_request()
    fbk = group.get_next_feedback()
    if fbk is None:
        raise RuntimeError("Could not get feedback")

    traj = create_trajectory([0, duration], [fbk.position[0], end])
    t0 = time()
    t1 = t0 + duration
    t = t0

    while t < t1:
        t = time()
        relative_time = t - t0
        p, v, _ = traj.get_state(relative_time)
        cmd.position = p
        cmd.velocity = v
        group.send_command(cmd)
        sleep(0.01)


def is_obstructed(group_fbk, thresh, scale, last_obstruction_time, obstruction_deadzone):
    if (time() - last_obstruction_time) < obstruction_deadzone:
        # If in the (small) obstruction deadzone time frame, do not report having encountered an obstruction.
        return False
    effort = group_fbk.effort[0]
    return scale * effort > thresh


lookup = hebi.Lookup()
sleep(2)

group = lookup.get_group_from_names(['HEBI'], ['X5-1'])
cmd = hebi.GroupCommand(1)


# Tweakable parameters
# Note - this is in degrees
full_sweep_angle = 115.0
sweep_duration = 1.0

TRAJECTORY_PERIOD = sweep_duration * 2.0
OBSTRUCTION_THRESHHOLD = 0.8
OBSTRUCTION_DEADZONE = 0.075

group.feedback_frequency = 100.0


global starting_position
global traj_offset_time
global full_trajectory

global commanding_away_from_obstruction
global last_obstruction_time
global obstructed_trajectory
global obstructed_time_scale

commanding_away_from_obstruction = False
last_obstruction_time = 0.0

# `0` if an obstruction was found when sweeping from start->end
# `1` if an obstruction was found when sweeping from end->start
global obstructed_arc_segment


def sweeping_handler(group_fbk):
    global traj_offset_time
    relative_time = fmod(time() - traj_offset_time, TRAJECTORY_PERIOD)

    if relative_time < 1.0:
        # Moving from starting waypoint to end of arc
        effort_scale = 1.0
    else:
        # Moving back to starting arc position
        effort_scale = -1.0

    global commanding_away_from_obstruction
    global last_obstruction_time

    if not commanding_away_from_obstruction and is_obstructed(group_fbk, OBSTRUCTION_THRESHHOLD, effort_scale, last_obstruction_time, OBSTRUCTION_DEADZONE):
        # Replan from current position to arc endpoint last visited
        global obstructed_trajectory
        global obstructed_time_scale
        global obstructed_arc_segment

        last_obstruction_time = time()
        commanding_away_from_obstruction = True

        current_position = group_fbk.position[0]
        full_sweep_rad = radians(full_sweep_angle)

        if relative_time < 1.0:
            # Move back to starting (arc) waypoint
            obstructed_arc_segment = 0
            end_position = first_start_position
        else:
            # Move back to ending (arc) waypoint
            obstructed_arc_segment = 1
            end_position = first_start_position + full_sweep_rad

        obstructed_time_scale = abs((current_position - end_position) / full_sweep_rad) * sweep_duration
        obstructed_trajectory = create_trajectory([0, obstructed_time_scale], [current_position, end_position])
        print(f'Detected obstruction near {current_position}!\nReducing sweeping to {current_position}->{end_position}')

    regular_path_planner = True

    if commanding_away_from_obstruction:
        obstruction_end_time = last_obstruction_time + obstructed_trajectory.duration
        now_time = time()
        if now_time > obstruction_end_time:
            # fall back into normal trajectory now
            commanding_away_from_obstruction = False
            # Adjust `traj_offset_time` to pick the right time point in the trajectory path
            if obstructed_arc_segment == 0:
                traj_offset_time = now_time
                relative_time = 0.0
            else:
                # Offset by `sweep_duration` to modify the time as necessary
                traj_offset_time = now_time + sweep_duration
                relative_time = sweep_duration
        else:
            regular_path_planner = False
            relative_time = (now_time - last_obstruction_time)

    if regular_path_planner:
        p, v, _ = full_trajectory.get_state(relative_time)
    else:
        p, v, _ = obstructed_trajectory.get_state(relative_time)

    cmd.position = p
    cmd.velocity = v
    group.send_command(cmd)


# Starting position will be defined as the position when starting the script
group.send_feedback_request()
fbk = group.get_next_feedback()

if fbk is None:
    print('Could not receive feedback from module')
    exit(1)

first_start_position = fbk.position[0]
starting_position = first_start_position

# This is the full (unobstructed) trajectory, which won't ever change
start_arc = first_start_position
end_arc = first_start_position + radians(full_sweep_angle)
full_trajectory = create_trajectory([0, sweep_duration, sweep_duration * 2],
                                    [start_arc, end_arc, start_arc])

print(f'sweeping from {first_start_position}->{end_arc}')
# Note: `first_start_position` is considered one of the endpoints of the arc - NOT the midpoint.

traj_offset_time = time()
group.add_feedback_handler(sweeping_handler)

# Runs for a minute
sleep(60.0)

smooth_move_to(group, cmd, first_start_position)
