#!/usr/bin/env python3

from time import sleep
import numpy as np
import hebi

# Lookup initialization
lookup = hebi.Lookup()
sleep(2)

# Setup Mobile IO input device
phone = hebi.util.create_mobile_io(
  lookup=lookup,
  family="HEBI",
  name="mobileIO")

if phone is None:
  raise RuntimeError("Could not find Mobile IO device")
phone.set_button_mode(1, 'momentary')
phone.set_button_mode(2, 'toggle')
phone.update()

# Setup Arm
arm = hebi.arm.create(
  lookup=lookup,
  families=["Arm"],
  names=['J1_base',
         'J2_shoulder',
         'J3_elbow',
         'J4_wrist1',
         'J5_wrist2',
         'J6_wrist3'],
  hrdf_file="hrdf/A-2085-06.hrdf")

# Increase feedback frequency since we're calculating velocities at the
# high level for damping. Going faster can help reduce a little bit of
# jitter for fast motions, but going slower (100 Hz) also works just fine
# for most applications.
arm.group.feedback_frequency = 200.0

# Add Impedance controller and setup gains
impedance_controller = hebi.arm.ImpedanceController()
arm.add_plugin(impedance_controller)

# example: hold position, but allow rotation around end-effector position
impedance_controller.set_damper_gains(5, 5, 5, 0, 0, 0)
impedance_controller.set_spring_gains(500, 500, 500, 0, 0, 0)
impedance_controller.gains_in_end_effector_frame = True

# Double the effort gains from their default values, to make the arm more sensitive for tracking force.
# TODO

enable_logging = True
goal = hebi.arm.Goal(arm.size)

# Start background logging
if enable_logging:
  arm.group.start_log(
    directory='logs',
    name='logFile',
    mkdirs=True)

print('Commanded gravity-compensated zero force to the arm.')
print('  b2 - Toggles an impedance controller on/off:')
print('          ON  - Apply controller based on current position [blue]')
print('          OFF - Go back to gravity-compensated mode [green]')
print('  b1 - Exits the demo')

# Run main demo
phone.set_led_color('green')
controller_on = False
disabled = np.ones(arm.size) * np.nan
while not phone.get_button_state(1):

  if not arm.update():
    print("Failed to update arm")
    continue

  if phone.update(timeout_ms=0):
    b2 = bool(phone.get_button_state(2))
    if controller_on is not b2:
      controller_on = b2
      if controller_on:
        arm.set_goal(goal.clear().add_waypoint(position=arm.last_feedback.position))
        color = 'blue'
      else:
        arm.cancel_goal()
        color = 'green'
      phone.set_led_color(color, blocking=False)

  # In this example we want to run pure torque control, so we disable the pos/vel
  # commands. Typically these would be enabled, and impedance control would be used
  # to further improve tracking.
  arm.pending_command.position = disabled
  arm.pending_command.velocity = disabled
  arm.send()

phone.set_led_color('transparent')

if enable_logging:
  log_file = arm.group.stop_log()
  hebi.util.plot_logs(log_file, 'position', figure_spec=101)
  hebi.util.plot_logs(log_file, 'velocity', figure_spec=102)
  hebi.util.plot_logs(log_file, 'effort', figure_spec=103)
