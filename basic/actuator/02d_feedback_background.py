#!/usr/bin/env python3

import hebi
from time import sleep
from matplotlib import pyplot as plt

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_name = "Test Actuator"

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

# This is by default 100 Hz. Setting this to 5 Hz allows the console output to be reasonable.
group.feedback_frequency = 5.0

plt.ion()
f = plt.figure()
plt.title('Actuator Gyro Feedback')
plt.xlabel('Axis')
plt.ylabel('Angular Velocity (rad/sec)')
plt.ylim([-15, 15])
plt.grid(True)
# Start with 0 data first
plt.bar([0, 1, 2], [0, 0, 0])
plt.draw()

latest_gyro = [0, 0, 0]

def feedback_handler(group_feedback):
    global latest_gyro
    latest_gyro = group_feedback.gyro[0]

group.add_feedback_handler(feedback_handler)

bars = plt.bar([0, 1, 2], [0, 0, 0])
for _ in range(50):
    temp_gyro = latest_gyro.copy()
    for i in range(3):
        bars[i].set_height(temp_gyro[i])
    plt.pause(0.01)
    sleep(0.2)
