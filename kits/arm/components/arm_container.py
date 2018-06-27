import numpy as np
import hebi


class ArmContainer(object):

  def __init__(self, group, robot_model):
    self._group = group
    self._robot = robot_model
    self._masses = robot_model.masses

  @property
  def dof_count(self):
    return self._robot.dof_count

  @property
  def group(self):
    return self._group

  @property
  def robot(self):
    return self._robot

  def get_efforts(self, feedback):
    """
    Gets the torques whih approximately balance out the effect
    of gravity on the arm
    """
    gravity = -1.0*feedback[0].accelerometer
    gravity_norm = np.linalg.norm(gravity)
    if gravity_norm > 0.0:
      gravity *= 9.81/gravity_norm
    else:
      return np.zeros(self._robot.dof_count, dtype=np.float64)

    num_dof = self._robot.dof_count
    num_frames = self._robot.get_frame_count('com')
    jacobians = self._robot.get_jacobians('com', feedback.position)

    masses = self._masses
    comp_torque = np.asmatrix(np.zeros((num_dof, 1), dtype=np.float64))
    wrench_vec = np.zeros((6, 1), dtype=np.float64)

    for i in range(num_frames):
      # Set translational part
      for j in range(3):
        wrench_vec[j, 0] = -gravity[j] * masses[i]
      comp_torque += jacobians[i].T * wrench_vec

    return comp_torque.A1


def create_3_dof():
  lookup = hebi.Lookup()

  # You can modify the names here to match modules found on your network
  module_family = 'HEBI'
  module_names = ['base', 'shoulder', 'elbow']

  from time import sleep
  sleep(2)
  arm = lookup.get_group_from_names([module_family], module_names)

  if arm is None:
    print('\nCould not find arm group: Did you forget to set the module family and names?')
    print('Searched for modules named:')
    print("{0} with family '{1}'".format(
      ', '.join(["'{0}'".format(entry) for entry in module_names]), module_family))

    print('Modules on the network:')
    for entry in lookup.entrylist:
      print(entry)
    else:
      print('[No Modules Found]')
    exit(1)

  model = hebi.robot_model.RobotModel()
  model.add_actuator('X5-4')
  model.add_bracket('X5-LightBracket', 'right')
  model.add_actuator('X5-4')
  model.add_link('X5', extension=0.18, twist=np.pi)
  model.add_actuator('X5-4')
  model.add_link('X5', extension=0.18, twist=0)

  assert arm.size == model.dof_count
  return ArmContainer(arm, model)
