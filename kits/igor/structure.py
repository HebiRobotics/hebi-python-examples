from . import DemoUtils, Joystick
from .config import Igor2Config
import hebi

class Chassis(object):

  def __init__(self):
    pass


class Arm(object):

  def __init__(self):
    pass


class Leg(object):

  def __init__(self):
    pass


class Igor(object):

  def __create_group(self):
    """
    Continuously attempt to create the Igor II group
    """
    lookup = hebi.Lookup()
    from time import sleep
    sleep(2.0)

    if self.__has_camera:
      def connect():
        ret = lookup.get_group_from_names([self.__config.family], self.__config.module_names)
        assert ret != None
        return ret
    else:
      def connect():
        ret = lookup.get_group_from_names([self.__config.family], self.__config.module_names_no_cam)
        assert ret != None
        return ret
    #self.__group = DemoUtils.retry_on_error(connect)

  def __find_joystick(self):
    def find_joystick():
      # TODO: Don't hardcode first joystick found
      return Joystick.at_index(0)
    self.__joy = DemoUtils.retry_on_error(find_joystick)

  def __init__(self, has_camera=True):
    self.__config = Igor2Config()
    self.__has_camera = has_camera
    self.__started = False
    self.__joy = None
    self.__group = None

    from threading import Lock
    self.__state_lock = Lock()

  def start(self):
    self.__state_lock.acquire()
    if self.__started:
      self.__state_lock.release()
      return
    self.__create_group()
    self.__started = True
    self.__state_lock.release()

