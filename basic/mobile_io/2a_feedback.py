#!/usr/bin/env python3

import hebi
from time import sleep, time
from matplotlib import pyplot as plt

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "HEBI"
module_name = "mobileIO"

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

group.feedback_frequency = 50.0

group_feedback = hebi.GroupFeedback(group.size)

plt.ion()
f = plt.figure()
plt.title('Mobile IO Gyro Feedback')
plt.xlabel('Axis')
plt.ylabel('Angular Velocity (rad/sec)')
plt.ylim([-15, 15])
plt.grid(True)
# Start with 0 data first
plt.bar([0, 1, 2], [0, 0, 0])
plt.draw()

duration = 10.0
start_time = time()
end_time = start_time + duration
current_time = start_time

while current_time < end_time:
    if group.get_next_feedback(reuse_fbk=group_feedback) is None:
        print('Failed to get feedback')
        continue
    gyro = group_feedback.gyro[0]

    plt.bar([0, 1, 2], gyro)
    plt.pause(0.00001)

    current_time = time()
