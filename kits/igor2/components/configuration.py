import os

class Igor2Config(object):
  """
  Used when starting up Igor II.
  """

  def __init__(self, imitation=True):
    self.__module_names = ['wheel1', 'wheel2',
                           'hip1', 'knee1',
                           'hip2', 'knee2',
                           'base1', 'shoulder1', 'elbow1', 'wrist1',
                           'base2', 'shoulder2', 'elbow2', 'wrist2',
                           'camTilt']
    self.__family = 'Igor II'
    here_path = os.path.dirname(__file__)
    self.__default_gains = os.path.join(here_path, 'igorGains.xml')
    self.__default_gains_no_cam = os.path.join(here_path, 'igorGains_noCamera.xml')
    self.__imitation = imitation

  @property
  def module_names(self):
    """
    :return:
    :rtype:  list
    """
    return self.__module_names

  @property
  def module_names_no_cam(self):
    """
    :return:
    :rtype:  list
    """
    return self.__module_names[0:-1]

  @property
  def family(self):
    """
    :return:
    :rtype:  str
    """
    return self.__family

  @property
  def gains_xml(self):
    """
    :return:
    :rtype:  str
    """
    return self.__default_gains

  @property
  def gains_no_camera_xml(self):
    """
    :return:
    :rtype:  str
    """
    return self.__default_gains_no_cam

  @property
  def is_imitation(self):
    """
    :return:
    :rtype:  bool
    """
    return self.__imitation
