import hebi
from . import DemoUtils, Joystick
from time import sleep
from functools import partial as funpart


# ------------------------------------------------------------------------------
# Joystick functions
# ------------------------------------------------------------------------------


def on_error_find_joystick(group, command):
  command.led.color = 'white'
  group.send_command(command)
  sleep(0.1)
  command.led.color = 'magenta'
  group.send_command(command)
  sleep(0.1)


def get_first_joystick():
  for i in range(Joystick.joystick_count()):
    try:
      return Joystick.at_index(i)
    except:
      pass


def find_joystick(igor):
  """
  Helper function to continuously search for a joystick and return it
  """
  group = igor.group
  group_command = hebi.GroupCommand(group.size)

  if igor.config.is_imitation:
    return DemoUtils.retry_on_error(get_first_joystick)
  else:
    on_error = funpart(on_error_find_joystick, group, group_command)
    return DemoUtils.retry_on_error(get_first_joystick, on_error)


# ------------------------------------------------------------------------------
# Gains functions
# ------------------------------------------------------------------------------


def load_gains(igor):
  group = igor.group

  # Bail out if group is imitation
  if igor.config.is_imitation:
    return

  gains_command = hebi.GroupCommand(group.size)
  sleep(0.1)

  try:
    if igor.has_camera:
      gains_command.read_gains(igor.config.gains_xml)
    else:
      gains_command.read_gains(igor.config.gains_no_camera_xml)
  except Exception as e:
    print('Warning - Could not load gains: {0}'.format(e))
    return

  # Send gains multiple times
  for i in range(3):
    group.send_command(gains_command)
    sleep(0.1)


# ------------------------------------------------------------------------------
# Group utility functions
# ------------------------------------------------------------------------------


def set_command_subgroup_pve(group_command, pos, vel, effort, indices):
  """
  Set position, velocity, and effort for certain modules in a group

  :param group_command: the GroupCommand instance into which the values will be sent
  :param pos:           the whole group's position array
  :param vel:           the whole group's velocity array
  :param effort:        the whole group's effort array. May be ``None``
  :param indices:       the indices in the group which will be modified
  """
  idx = 0
  if effort is None:
    for i in indices:
      cmd = group_command[i]
      cmd.position = pos[idx]
      cmd.velocity = vel[idx]
      idx = idx + 1
  else:
    for i in indices:
      cmd = group_command[i]
      cmd.position = pos[idx]
      cmd.velocity = vel[idx]
      cmd.effort = effort[idx]
      idx = idx + 1


from hebi.util import create_imitation_group
def create_group(config, has_camera):
  """
  Used by :class:`.structure.Igor` to create the group to interface with modules

  :param config:     The runtime configuration
  :type config:      .config.Igor2Config
  :param has_camera: 
  :type has_camera:  bool
  """
  imitation = config.is_imitation

  if imitation:
    if has_camera:
      num_modules = len(config.module_names)
    else:
      num_modules = len(config.module_names_no_cam)

    return create_imitation_group(num_modules)
  else:
    if has_camera:
      names = config.module_names
    else:
      names = config.module_names_no_cam
    families = [config.family]
    lookup = hebi.Lookup()

    def connect():
      group = lookup.get_group_from_names(families, names)
      if group == None:
        raise RuntimeError()
      elif group.size != len(names):
        raise RuntimeError()
      return group

    # Let the lookup object discover modules, before trying to connect
    sleep(2.0)
    return DemoUtils.retry_on_error(connect)


# ------------------------------------------------------------------------------
# Misc functions
# ------------------------------------------------------------------------------


import sys, threading
# Used for Igor background controller thread
if sys.version_info[0] == 3:
  is_main_thread_active = lambda: threading.main_thread().is_alive()
else:
  is_main_thread_active = lambda: any((i.name == "MainThread") and i.is_alive() for i in threading.enumerate())