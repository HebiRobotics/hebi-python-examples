from functools import partial as funpart
from util import math_utils

# ------------------------------------------------------------------------------
# Arm Event Handlers
# ------------------------------------------------------------------------------


def arm_x_vel_event(igor, in_deadzone_func, ts, axis_value):
  """
  Event handler when left stick Y-Axis motion occurs.
  This event handler will set the given arm's velocity in the X axis to
  a value proportional to the given input `axis_value` when outside the
  joystick deadzone. When the value is within the joystick deadzone, the
  arm's velocity in the X axis is set to zero.
  (Left Stick Y-Axis event handler)

  :param igor:             (bound parameter)
  :param in_deadzone_func: (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  if in_deadzone_func(axis_value):
    igor.left_arm.set_x_velocity(0.0)
    igor.right_arm.set_x_velocity(0.0)
  else:
    scale = -0.4
    axis_value = scale*axis_value
    igor.left_arm.set_x_velocity(scale*axis_value)
    igor.right_arm.set_x_velocity(scale*axis_value)


def arm_y_vel_event(igor, in_deadzone_func, ts, axis_value):
  """
  Event handler when left stick X-Axis motion occurs.
  This event handler will set the given arm's velocity in the Y axis to
  a value proportional to the given input `axis_value` when outside the
  joystick deadzone. When the value is within the joystick deadzone, the
  arm's velocity in the Y axis is set to zero.
  (Left Stick X-Axis event handler)

  :param igor:             (bound parameter)
  :param in_deadzone_func: (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  if in_deadzone_func(axis_value):
    igor.left_arm.set_y_velocity(0.0)
    igor.right_arm.set_y_velocity(0.0)
  else:
    scale = -0.4
    axis_value = scale*axis_value
    igor.left_arm.set_y_velocity(axis_value)
    igor.right_arm.set_y_velocity(axis_value)


def arm_z_vel_event_l(igor, ts, axis_value):
  """
  Event handler when left trigger of joystick has its axis value changed
  (Left Trigger Axis event handler)

  :param igor:       (bound parameter)
  :param ts:         (ignored)
  :param axis_value: [-1,1] value of the axis
  """
  # Left Trigger
  # map [-1,1] to [0,1]
  axis_value = -0.2*(axis_value*0.5+0.5)
  set_arm_vel_z(igor.left_arm, axis_value)
  set_arm_vel_z(igor.right_arm, axis_value)


def arm_z_vel_event_r(igor, ts, axis_value):
  """
  Event handler when right trigger of joystick has its axis value changed
  (Right Trigger Axis event handler)

  :param igor:       (bound parameter)
  :param ts:         (ignored)
  :param axis_value: [-1,1] value of the axis
  """
  # Right Trigger
  # map [-1,1] to [0,1]
  axis_value = 0.2*(axis_value*0.5+0.5)
  set_arm_vel_z(igor.left_arm, axis_value)
  set_arm_vel_z(igor.right_arm, axis_value)


def zero_arm_z_event(igor, both_triggers_released, ts, pressed):
  """
  Event handler when left or right trigger of joystick is pressed or released.
  This event handler will zero the Z axis velocity of the arms, if both joysticks
  are not being pressed. If this condition is not satisfied, then this function
  does nothing.

  :param igor:                   (bound parameter)
  :param both_triggers_released: (bound parameter)
  :param ts:                     (ignored)
  :param pressed:                (ignored)
  """
  if both_triggers_released():
    igor.left_arm.set_z_velocity(0.0)
    igor.right_arm.set_z_velocity(0.0)


def wrist_vel_event(igor, ts, hx, hy):
  """
  Event handler when d-pad is pressed.
  This event handler will set the velocity of the wrists.

  :param igor:  (bound parameter)
  :param ts:    (ignored)
  :param hx:
  :param hy:
  :return:
  """
  l_arm = igor.left_arm
  r_arm = igor.right_arm
  if hy == 0:
    l_arm.set_wrist_velocity(0.0)
    r_arm.set_wrist_velocity(0.0)
  else:
    joy_low_pass = 0.95
    vel = (joy_low_pass*l_arm.wrist_velocity)+hy*(1.0-joy_low_pass)*0.25
    l_arm.set_wrist_velocity(vel)
    r_arm.set_wrist_velocity(vel)


# ------------------------------------------------------------------------------
# Chassis Event Handlers
# ------------------------------------------------------------------------------


def chassis_velocity_event(igor, in_deadzone_func, ts, axis_value):
  """
  Event handler when right stick Y-Axis motion occurs.

  :param igor:             (bound parameter)
  :param in_deadzone_func: (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  chassis = igor.chassis
  if in_deadzone_func(axis_value):
    chassis.set_directional_velocity(0.0)
  else:
    joy_scale = -0.5
    dead_zone = igor.joystick_dead_zone
    value = joy_scale*(axis_value-(dead_zone*math_utils.sign(axis_value)))
    chassis.set_directional_velocity(value)


def chassis_yaw_event(igor, in_deadzone_func, ts, axis_value):
  """
  Event handler when right stick X-Axis motion occurs.

  :param igor:             (bound parameter)
  :param in_deadzone_func: (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  chassis = igor.chassis
  if in_deadzone_func(axis_value):
    chassis.set_yaw_velocity(0.0)
  else:
    scale = 25.0
    dead_zone = igor.joystick_dead_zone
    wheel_radius = igor.wheel_radius
    wheel_base = igor.wheel_base
    value = (scale * wheel_radius / wheel_base) * (axis_value - dead_zone * math_utils.sign(axis_value))
    chassis.set_yaw_velocity(value)


# ------------------------------------------------------------------------------
# Misc Event Handlers
# ------------------------------------------------------------------------------


def stance_height_triggers_event(igor, joy, vel_calc, ts, axis_value):
  """

  :param igor:
  :param joy:
  :param vel_calc:
  :param ts:
  :param axis_value:
  """
  # Ignore this if `OPTIONS` is pressed
  if joy.get_button('OPTIONS'):
    return

  val = vel_calc()
  igor.left_leg.set_knee_velocity(val)
  igor.right_leg.set_knee_velocity(val)


def stance_height_event(igor, vel_calc, ts, pressed):
  """
  Event handler when `OPTIONS` button is pressed or released.

  :param igor:     (bound parameter)
  :param vel_calc: (bound parameter)
  :param ts:
  :param pressed:
  """
  if pressed:
    igor.left_leg.set_knee_velocity(1.0)
    igor.right_leg.set_knee_velocity(1.0)
    if igor.left_leg.knee_angle > 2.5:
      # TODO: implement restart
      pass
      #igor.request_restart()
      #igor.request_stop()
  else:
    val = vel_calc()
    igor.left_leg.set_knee_velocity(val)
    igor.right_leg.set_knee_velocity(val)


def balance_controller_event(igor, ts, pressed):
  """
  Event handler when `TOUCHPAD` button is pressed or released.

  :param igor:
  :param ts:
  :param pressed:
  """
  igor.set_balance_controller_state(not pressed)


def quit_session_event(igor, ts, pressed):
  """
  Event handler when `SHARE` button is pressed or released.
  When the button is pressed,
  and the robot is ready (in the "started" state),
  the session will begin to quit.

  :param igor:
  :param ts:
  :param pressed:
  """
  if pressed and igor.started:
    igor.request_stop()


# ------------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------------


def set_arm_vel_z(arm, val):
  """
  Set the Z axis velocity of the given arm.
  If the velocity is less than 1e-4, this function does nothing.

  :param arm:
  :param val: The velocity
  """
  # Ignore small values from being set (noise)
  if abs(val) > 1e-4:
    arm.set_z_velocity(val)


def both_triggers_released(joy):
  """
  :return: ``True`` if both the left and right triggers are not being pressed
  """
  left = joy.get_button('LEFT_TRIGGER')
  right = joy.get_button('RIGHT_TRIGGER')
  return not (left or right)


# ------------------------------------------------------------------------------
# Register event handlers
# ------------------------------------------------------------------------------


def register_igor_event_handlers(igor):
  """
  Registers all Igor joystick event handlers
  """

  joystick = igor.joystick

  # ----------------------------------------------
  # Functions to be passed to event handlers below

  arm_x_deadzone = lambda val: abs(val) <= igor.joystick_dead_zone*3.0
  arm_y_deadzone = lambda val: abs(val) <= igor.joystick_dead_zone

  def stance_height_calc():
    l_val = joystick.get_axis('LEFT_TRIGGER')
    r_val = joystick.get_axis('RIGHT_TRIGGER')
    d_ax = l_val-r_val
    if abs(d_ax) > igor.joystick_dead_zone:
      return 0.5*d_ax
    return 0.0

  # The current joystick used is not a global state, so we need to wrap it here
  both_triggers = lambda: both_triggers_released(joystick)

  # ----------------------------------------------------------------------
  # Functions which have bound parameters, in order to have right function
  # signature for event handlers

  # -----------------------
  # Left Arm event handlers

  # Reacts to left stick Y-axis
  arm_x_vel = funpart(arm_x_vel_event, igor, arm_x_deadzone)
  joystick.add_axis_event_handler('LEFT_STICK_Y', arm_x_vel)

  # Reacts to left stick X-axis
  arm_y_vel = funpart(arm_y_vel_event, igor, arm_y_deadzone)
  joystick.add_axis_event_handler('LEFT_STICK_X', arm_y_vel)

  # Reacts to left trigger axis
  arm_z_vel_lt = funpart(arm_z_vel_event_l, igor)
  joystick.add_axis_event_handler('LEFT_TRIGGER', arm_z_vel_lt)

  # Reacts to right trigger axis
  arm_z_vel_rt = funpart(arm_z_vel_event_r, igor)
  joystick.add_axis_event_handler('RIGHT_TRIGGER', arm_z_vel_rt)

  # ------------------------
  # Both Arms event handlers

  # Reacts to triggers pressed/released
  zero_arm_z = funpart(zero_arm_z_event, igor, both_triggers)
  joystick.add_button_event_handler('LEFT_TRIGGER', zero_arm_z)
  joystick.add_button_event_handler('RIGHT_TRIGGER', zero_arm_z)

  # Reacts to D-Pad pressed/released
  wrist_vel = funpart(wrist_vel_event, igor)
  #joystick.add_dpad_event_handler(wrist_vel)

  # ----------------------
  # Chassis event handlers

  # Reacts to right stick Y-axis
  chassis_velocity = funpart(chassis_velocity_event, igor, arm_y_deadzone)
  joystick.add_axis_event_handler('RIGHT_STICK_Y', chassis_velocity)

  # Reacts to right stick X-axis
  chassis_yaw = funpart(chassis_yaw_event, igor, arm_y_deadzone)
  joystick.add_axis_event_handler('RIGHT_STICK_X', chassis_yaw)

  # -------------
  # Stance height

  stance_height_trigger = funpart(stance_height_triggers_event, igor, joystick, stance_height_calc)
  joystick.add_axis_event_handler('LEFT_TRIGGER', stance_height_trigger)
  joystick.add_axis_event_handler('RIGHT_TRIGGER', stance_height_trigger)

  stance_height = funpart(stance_height_event, igor, stance_height_calc)
  joystick.add_button_event_handler('OPTIONS', stance_height)

  # ---------------
  # Toggle Balancer

  balance_controller = funpart(balance_controller_event, igor)
  joystick.add_button_event_handler('TOUCHPAD', balance_controller)

  # ------------
  # Quit Session

  quit_session = funpart(quit_session_event, igor)
  joystick.add_button_event_handler('SHARE', quit_session)

  # ------------------------------
  # Camera specific event handlers

  if igor.has_camera:
    # Implement your own functionality here
    def c_set_cam(ts, pressed):
      pass
    def sq_set_cam(ts, pressed):
      pass
    def t_set_cam(ts, pressed):
      pass
    def x_set_cam(ts, pressed):
      pass

    joystick.add_button_event_handler('CIRCLE', c_set_cam)
    joystick.add_button_event_handler('SQUARE', sq_set_cam)
    joystick.add_button_event_handler('TRIANGLE', t_set_cam)
    joystick.add_button_event_handler('X', x_set_cam)
