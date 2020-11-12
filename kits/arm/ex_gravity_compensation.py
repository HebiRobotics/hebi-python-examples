#!/usr/bin/env python3

from time import sleep, time

import hebi

# Example parameters
enable_logging = True
duration = 10  # [s]

# Lookup initialization
lookup = hebi.Lookup()
sleep(2)

# Setup Arm
arm = hebi.arm.create(
  families=["Arm"],
  names=['J1_base',
         'J2_shoulder',
         'J3_elbow',
         'J4_wrist1',
         'J5_wrist2',
         'J6_wrist3'],
  lookup=lookup,
  hrdf_file="hrdf/A-2085-06.hrdf")

# Start background logging
if enable_logging:
  arm.group.start_log(
    directory='logs',
    name='logFile',
    mkdirs=True)

print('Commanding gravity-compensating torques')
t0 = time()
while time() - t0 < duration:

  if not arm.update():
    print("Failed to update arm")
    continue

  arm.send()

print('Stopped example')
if enable_logging:
  log_file = arm.group.stop_log()
  hebi.util.plot_logs(log_file, 'position', figure_spec=101)
  hebi.util.plot_logs(log_file, 'velocity', figure_spec=102)
  hebi.util.plot_logs(log_file, 'effort', figure_spec=103)
