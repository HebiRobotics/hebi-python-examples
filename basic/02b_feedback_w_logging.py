#!/usr/bin/env python3

import hebi
from time import sleep, time
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

# Live Visualization
# Starts logging in the background. Note that logging can be enabled at any time, and that it does not negatively
# affect the performance of your running programs.
group.start_log('dir', 'logs', mkdirs=True)

print('Use Scope to command the module and make it move...')
plt.ion()
f = plt.figure()
plt.ylim([-5, 5])
plt.title('Module Output Velocity')
plt.ylabel('Angular Velocity (rad/sec)')
plt.grid(True)

duration = 10.0
start_time = time()
end_time = start_time + duration
current_time = start_time

while current_time < end_time:
    current_time = time()
    fbk = group.get_next_feedback()
    plt.bar(current_time, fbk.velocity)
    plt.pause(0.00001)

print('All done!')

# Stops background logging and converts the logged data into readable data that can be easily plotted.
log = group.stop_log()
log.load()  # load into memory for easier multi-plotting

# Offline Visualization
if log is not None:
    # Plot the logged position feedback
    hebi.util.plot_logs(log, 'position', figure_spec=101)
    # Plot the logged velocity feedback
    hebi.util.plot_logs(log, 'velocity', figure_spec=102)
