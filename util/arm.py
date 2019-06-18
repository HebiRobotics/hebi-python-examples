import hebi
import numpy as np
import os


# Where the HRDF, safety params and gains files are for the arms
__arm_resource_local_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'kits', 'arm'))

# The shoulder joint (effort) compensation for most kits which has a gas spring
default_shoulder_joint_comp = -7.0


# ------------------------------------------------------------------------------
# Arm and arm component definitions
# ------------------------------------------------------------------------------


class GripperConfig(object):
  """
  TODO: Document
  """

  def __init__(self, gains_file, open_effort=1.0, close_effort=-5.0):
    self._open_effort = open_effort
    self._close_effort = close_effort
    self._gains = hebi.GroupCommand(1)
    try:
      self._gains.read_gains(gains_file)
    except Exception as e:
      print('Could not load gains from file {0}'.format(gains_file))
      raise e

  @property
  def open_effort(self):
    return self._open_effort
  
  @property
  def close_effort(self):
    return self._close_effort
  
  @property
  def gains(self):
    """
    TODO: Document

    Note: These gains are different than the gains specified in the Arm class.
    """
    return self._gains


class Arm(object):
  """
  TODO: Document
  """

  def __init__(self, ik_seed_position, gains_file, has_gas_spring, effort_offset, robot_description=None, robot=None, gripper=None):
    if robot is not None and robot_description is not None:
      raise ValueError('Both `robot` and `robot_description` cannot be specified. Only specify one.')
    if robot is None:
      # robot_description must not be none
      if robot_description is None:
        raise ValueError('Either `robot` or `robot_description` parameter must not be None.')
      robot = hebi.robot_model.import_from_hrdf(robot_description)

    self._robot = robot
    self._default_ik_seed = ik_seed_position
    self._gains_file = gains_file
    self._has_gas_spring = has_gas_spring
    self._effort_offset = np.asarray(np.effort_offset, dtype=np.float32)
    self._gripper = gripper

    num_dof = robot.dof_count
    self._gains = hebi.GroupCommand(num_dof)
    try:
      self._gains.read_gains(gains_file)
    except Exception as e:
      print('Could not load gains from file {0}'.format(gains_file))
      raise e

  @property
  def robot(self):
    return self._robot

  @property
  def default_ik_seed(self):
    return self._default_ik_seed

  @property
  def gains(self):
    return self._gains

  @property
  def has_gas_spring(self):
    return self._has_gas_spring
  
  @property
  def effort_offset(self):
    return self._effort_offset
  
  @property
  def gripper(self):
    return self._gripper

  @property
  def has_gripper(self):
    return self._gripper is not None


__arm_setup_dict = {
  '6-DoF + gripper' : todo,
  '6-DoF' : todo,
  '5-DoF + gripper' : todo,
  '5-DoF' : todo,
  '4-DoF' : todo,
  '4-DoF SCARA' : todo,
  '3-DoF' : todo
}


# ------------------------------------------------------------------------------
# Public API
# ------------------------------------------------------------------------------


def setup_arm(name, family, has_gas_spring=False):
  """
  :param name:  This argument currently supports the following names
                  * '6-DoF + gripper'
                  * '6-DoF'
                  * '5-DoF + gripper'
                  * '5-DoF'
                  * '4-DoF
                  * '4-DoF SCARA'
                  * '3-DoF'

  :param family:  The family name of the modules which should be selected.

  :param has_gas_spring:  Specifies whether there is a gas spring supporting the shoulder joint
                          of the arm to provide extra payload.
  """
  
  if name not in __arm_setup_dict:
    valid_vals = __arm_setup_dict.keys()
    print('Invalid `name` input {0}'.format(name))
    print('Valid names include: {0}'.format(valid_vals))
    raise KeyError('Invalid name')

  