from . import DemoUtils, Joystick
from .config import Igor2Config
import hebi

from time import sleep
from functools import partial as funpart


class Chassis(object):

  def __init__(self):
    pass


class Arm(object):

  def __init__(self, name):
    self.__name = name

  def set_x_velocity(self, value):
    print('{0} Velocity(x): {1}'.format(self.__name, value))

  def set_y_velocity(self, value):
    print('{0} Velocity(y): {1}'.format(self.__name, value))

  def set_z_velocity(self, value):
    print('{0} Velocity(z): {1}'.format(self.__name, value))


class Leg(object):

  def __init__(self):
    pass


# ------------------------------------------------------------------------------
# Functions to be wrapped by Igor class
# ------------------------------------------------------------------------------


def __set_arm_vel_z(arm, val):
  # Ignore small values from being set (noise)
  if abs(val) > 1e-4:
    arm.set_z_velocity(val)


def on_error_find_joystick(group, command):
  command.led.color = 'white'
  group.send_command(command)
  sleep(0.1)
  command.led.color = 'magenta'
  group.send_command(command)
  sleep(0.1)


def arm_z_vel_event_r(arm, ts, axis_value):
  # Right Trigger
  # map [-1,1] to [0,1]
  axis_value = axis_value*0.5 + 0.5
  __set_arm_vel_z(arm, 0.2*axis_value)


def arm_z_vel_event_l(arm, ts, axis_value):
  # Left Trigger
  # map [-1,1] to [0,1]
  axis_value = axis_value*0.5 + 0.5
  __set_arm_vel_z(arm, -0.2*axis_value)


def arm_z_vel_zero_condition(arm, condition):
  if condition():
    arm.set_z_velocity(0.0)


# ------------------------------------------------------------------------------
# Event Handlers for Igor class
# ------------------------------------------------------------------------------


class Igor(object):

# ------------------------------------------------
# Event Handlers
# ------------------------------------------------

  def __stance_height_event(self, ts, pressed):
    if pressed:
      # TODO: Soft shutdown procedure
      pass
    else:
      # TODO: Normal stance height control
      pass

  def __zero_arm_z_event(self, ts, pressed):
    if self.__both_triggers_released():
      self.__left_arm.set_z_velocity(0.0)
      self.__right_arm.set_z_velocity(0.0)

  def __balance_controller_event(self, ts, pressed):
    # "enqueue" this for the next Igor processing loop
    controller_enabled = not pressed
    pass


# ------------------------------------------------
# Private Functions
# ------------------------------------------------

  def __both_triggers_released(self):
    joy = self.__joy
    left = joy.get_button('LEFT_TRIGGER')
    right = joy.get_button('RIGHT_TRIGGER')
    return not (left or right)

  def __create_group(self):
    """
    Continuously attempt to create the Igor II group
    """
    lookup = hebi.Lookup()
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
      return Joystick.at_index(1)

    #group = self.__group
    #group_command = hebi.GroupCommand(group.size)
    #on_error = funpart(on_error_find_joystick, group, group_command)
    #joy = DemoUtils.retry_on_error(find_joystick, on_error)

    joy = DemoUtils.retry_on_error(find_joystick)

    # Functors
    def arm_x_vel_event(arm, ts, axis_value):
      if abs(axis_value) > self.__joy_dead_zone*3.0:
        scale = -0.4 # TODO: make into customizable parameter
        arm.set_x_velocity(scale*axis_value)
      else:
        arm.set_x_velocity(0.0)

    def arm_y_vel_event(arm, ts, axis_value):
      if abs(axis_value) > self.__joy_dead_zone:
        scale = -0.4 # TODO: make into customizable parameter
        arm.set_y_velocity(scale*axis_value)
      else:
        arm.set_y_velocity(0.0)

    l_arm_x_vel = funpart(arm_x_vel_event, self.__left_arm)
    l_arm_y_vel = funpart(arm_y_vel_event, self.__left_arm)
    l_arm_z_vel_rt = funpart(arm_z_vel_event_r, self.__left_arm)
    l_arm_z_vel_lt = funpart(arm_z_vel_event_l, self.__left_arm)
    r_arm_x_vel = funpart(arm_x_vel_event, self.__right_arm)
    r_arm_y_vel = funpart(arm_y_vel_event, self.__right_arm)
    r_arm_z_vel_rt = funpart(arm_z_vel_event_r, self.__right_arm)
    r_arm_z_vel_lt = funpart(arm_z_vel_event_l, self.__right_arm)

    # Register event handlers
    joy.add_button_event_handler('OPTIONS', self.__stance_height_event)
    joy.add_button_event_handler('LEFT_TRIGGER', self.__zero_arm_z_event)
    joy.add_button_event_handler('RIGHT_TRIGGER', self.__zero_arm_z_event)
    joy.add_axis_event_handler('LEFT_STICK_Y', l_arm_x_vel)
    joy.add_axis_event_handler('LEFT_STICK_Y', r_arm_x_vel)
    joy.add_axis_event_handler('LEFT_STICK_X', l_arm_y_vel)
    joy.add_axis_event_handler('LEFT_STICK_X', r_arm_y_vel)
    joy.add_axis_event_handler('LEFT_TRIGGER', l_arm_z_vel_lt)
    joy.add_axis_event_handler('LEFT_TRIGGER', r_arm_z_vel_lt)
    joy.add_axis_event_handler('RIGHT_TRIGGER', l_arm_z_vel_rt)
    joy.add_axis_event_handler('RIGHT_TRIGGER', r_arm_z_vel_rt)

    if self.__has_camera:
      def c_set_cam(ts, pressed):
        print('camera position ---> 1.4')
      def sq_set_cam(ts, pressed):
        print('camera position ---> 0.0')
      def t_set_cam(ts, pressed):
        print('camera velocity ---> -0.6')
      def x_set_cam(ts, pressed):
        print('camera velocity ---> 0.6')

      joy.add_button_event_handler('CIRCLE', c_set_cam)
      joy.add_button_event_handler('SQUARE', sq_set_cam)
      joy.add_button_event_handler('TRIANGLE', t_set_cam)
      joy.add_button_event_handler('X', x_set_cam)

    self.__joy = joy

  def __load_gains(self):
    group = self.__group
    group.feedback_frequency = 100.0

    gains_command = hebi.GroupCommand(group.size)
    sleep(1.0)

    if self.__has_camera:
      gains_command.load_gains(self.__config.gains_xml)
    else:
      gains_command.load_gains(self.__config.gains_no_camera_xml)
    
    # Send gains multiple times
    for i in range(3):
      group.send_command(gains_command)
      sleep(0.1)

  def __init__(self, has_camera=True):
    self.__config = Igor2Config()
    self.__has_camera = has_camera
    self.__started = False
    self.__joy = None
    self.__group = None

    self.__joy_dead_zone = 0.06

    self.__chassis = Chassis()
    self.__left_arm = Arm('Left')
    self.__right_arm = Arm('Right')
    self.__left_leg = Leg()
    self.__right_leg = Leg()

    from threading import Lock
    self.__state_lock = Lock()

  def start(self):
    self.__state_lock.acquire()
    if self.__started:
      self.__state_lock.release()
      return
    self.__create_group()
    self.__find_joystick()
    #self.__load_gains()
    self.__started = True
    self.__state_lock.release()

