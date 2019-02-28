

class HebiModuleController(object):
  
  def __init__(self, group):
    if group.size != 1:
      raise RuntimeError('Group must be of a single module')

    self._group = group


  def __fbk_handler(self, feedback):
    # TODO
    pass


  @feedback_frequency.setter
  def feedback_frequency(self, value):
    self._group.feedback_frequency = value

  @property
  def feedback_frequency(self):
    return self._group.feedback_frequency

  