

class Igor2Config(object):
  """
  Used when starting up Igor II.
  """

  def __init__(self):
    self.__module_names = ['wheel1', 'wheel2',
                           'hip1', 'knee1',
                           'hip2', 'knee2',
                           'base1', 'shoulder1', 'elbow1', 'wrist1',
                           'base2', 'shoulder2', 'elbow2', 'wrist2',
                           'camTilt']
    self.__module_names_no_cam = self.__module_names[0:-1]
    self.__family = 'Igor II'
    self.__default_gains = 'igorGains.xml'
    self.__default_gains_no_cam = 'igorGains_noCamera.xml'

  @property
  def module_names(self):
    return self.__module_names

  @property
  def module_names_no_cam(self):
    return self.__module_names_no_cam

  @property
  def family(self):
    return self.__family

  @property
  def gains_xml(self):
    return self.__default_gains

  @property
  def gains_no_camera_xml(self):
    return self.__default_gains_no_cam

