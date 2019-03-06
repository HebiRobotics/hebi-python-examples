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


def arm_z_vel_lower_event(igor, raise_btn_id, ts, button_value):
  """
  Event handler to respond to lowering arm (z direction) request

  :param igor:         (bound parameter)
  :param raise_btn_id: (bound parameter)
  :param ts:           (ignored)
  :param button_value: button pressed state
  """
  # If raise_btn_id is pressed too, just ignore this
  if igor.joystick.get_button(raise_btn_id):
    return

  if button_value:
    igor.left_arm.set_z_velocity(-0.2)
    igor.right_arm.set_z_velocity(-0.2)


def arm_z_vel_raise_event(igor, lower_btn_id, ts, button_value):
  """
  Event handler to respond to raising arm (z direction) request

  :param igor:         (bound parameter)
  :param ts:           (ignored)
  :param button_value: button pressed state
  """
  # If lower_btn_id is pressed too, just ignore this
  if igor.joystick.get_button(lower_btn_id):
    return

  if button_value:
    igor.left_arm.set_z_velocity(0.2)
    igor.right_arm.set_z_velocity(0.2)


# ------------------------------------------------------------------------------
# Wrist Event Handlers
# 
# Note: There are different event handlers depending on the type of controller
# ------------------------------------------------------------------------------


def wrist_vel_event__buttons(igor, scale, ts, value):
  l_arm = igor.left_arm
  r_arm = igor.right_arm
  if value:
    l_arm.set_wrist_velocity(scale)
    r_arm.set_wrist_velocity(scale)
  else:
    l_arm.set_wrist_velocity(0.0)
    r_arm.set_wrist_velocity(0.0)


def wrist_vel_event__axis(igor, deadzone_calc, ts, value):
  l_arm = igor.left_arm
  r_arm = igor.right_arm
  vel = deadzone_calc(value)
  l_arm.set_wrist_velocity(vel)
  r_arm.set_wrist_velocity(vel)


# ------------------------------------------------------------------------------
# Stance Handlers
# ------------------------------------------------------------------------------


def stance_height_triggers_event(igor, state, vel_calc, ts, value):
  # Ignore this if soft shutdown is currently being requested
  if not state.soft_shutdown_enabled:
    return

  val = vel_calc(value)
  igor.left_leg.set_knee_velocity(val)
  igor.right_leg.set_knee_velocity(val)


def soft_shutdown_event(igor, ts, pressed):
  if pressed:
    igor.left_leg.set_knee_velocity(1.0)
    igor.right_leg.set_knee_velocity(1.0)


# ------------------------------------------------------------------------------
# Chassis Event Handlers
# ------------------------------------------------------------------------------


def chassis_velocity_event(igor, in_deadzone_func, ts, axis_value):
  """
  Event handler to command chassis velocity

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
  Event handler to command chassis yaw

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


def balance_controller_event(igor, ts, pressed):
  """
  Event handler to a request to temporarily disable/enable the balance controller.

  Note: It is *highly* recommended to only use this when somebody is physically
  close enough to the robot to balance it manually. Otherwise, your robot will fall
  and potentially cause damage to itself or its surroundings!

  :param igor:
  :param ts:
  :param pressed:
  """
  pressed = bool(pressed)
  igor.set_balance_controller_state(not pressed)


def quit_session_event(igor, ts, pressed):
  """
  Event handler to react to the "quit" button being pressed or released.
  When the button is pressed, and the robot is ready (in the "started" state),
  the session will begin to quit.

  Note: Like the balance controller toggler, it is *highly* recommended to only
  request the session to quit when somebody is able to stabilize the robot
  during the shutdown procedure, to ensure that it "crouches" without falling
  over or hitting anything!

  :param igor:
  :param ts:
  :param pressed:
  """
  if pressed and igor.started:
    igor.request_stop()


# ------------------------------------------------------------------------------
# Binding event handler parameters
# ------------------------------------------------------------------------------


def bind_arm_x_deadzone(igor):
  return lambda val: abs(val) <= igor.joystick_dead_zone*3.0


def bind_arm_y_deadzone(igor):
  return lambda val: abs(val) <= igor.joystick_dead_zone


def deadzone_clip(igor):
  def calculate(val):
    if abs(val) <= igor.joystick_dead_zone:
      return 0.0
    else
      return val
  return calculate


def deadzone_clip_scaled(igor, dz_scale, out_scale):
  def calculate(val):
    if abs(val) <= igor.joystick_dead_zone * dz_scale:
      return 0.0
    else
      return val * out_scale
  return calculate


# ------------------------------------------------------------------------------
# Register event handlers
# ------------------------------------------------------------------------------


class IgorControlState(object):

  __slots__ = ('_balance_controller_enabled', '_soft_shutdown_enabled')
  def __init__(self):
    self._balance_controller_enabled = True
    self._soft_shutdown_enabled = False

  @property
  def soft_shutdown_enabled(self):
    return self._soft_shutdown_enabled
  
  @soft_shutdown_enabled.setter
  def soft_shutdown_enabled(self, value):
    self._soft_shutdown_enabled = value


def _add_event_handlers(igor, controller, controller_mapping):

  # Shortcut closures to get around having to use functool.partial
  def bind_to_method(method, *args):
    def bound_meth(ts, val):
      method(*args, ts, val)
    return bound_meth

  arm_x_deadzone_func = bind_arm_x_deadzone(igor)
  arm_y_deadzone_func = bind_arm_y_deadzone(igor)
  igor_state = IgorControlState()

  controller.add_button_event_handler(controller_mapping.quit, bind_to_method(quit_session_event, igor))
  controller.add_button_event_handler(controller_mapping.balance_controller_toggle, bind_to_method(balance_controller_event, igor))

  controller.add_axis_event_handler(controller_mapping.arm_vel_x, bind_to_method(arm_x_vel_event, igor, arm_x_deadzone_func))
  controller.add_axis_event_handler(controller_mapping.arm_vel_y, bind_to_method(arm_y_vel_event, igor, arm_y_deadzone_func))
  controller.add_axis_event_handler(controller_mapping.lower_arm, bind_to_method(arm_z_vel_lower_event, igor, controller_mapping.raise_arm))
  controller.add_axis_event_handler(controller_mapping.raise_arm, bind_to_method(arm_z_vel_raise_event, igor, controller_mapping.lower_arm))

  controller.add_axis_event_handler(controller_mapping.chassis_vel, bind_to_method(chassis_velocity_event, igor, arm_y_deadzone_func))
  controller.add_axis_event_handler(controller_mapping.chassis_yaw, bind_to_method(chassis_yaw_event, igor, arm_y_deadzone_func))

  # Stance height events are responded to differently by different kinds of controllers, thus requiring a bit more complex logic
  if controller_mapping.stance_height_control_strategy == 'TRIGGERS':
    stance_height_lower_axis_id = controller_mapping.stance_height[0]
    stance_height_raise_axis_id = controller_mapping.stance_height[1]

    lower_event = bind_to_method(stance_height_triggers_event, igor, state, deadzone_clip_scaled(igor, 5.0, 1.0))
    raise_event = bind_to_method(stance_height_triggers_event, igor, state, deadzone_clip_scaled(igor, 5.0, -1.0))

    controller.add_axis_event_handler(stance_height_lower_axis_id, lower_event)
    controller.add_axis_event_handler(stance_height_raise_axis_id, raise_event)
  elif controller_mapping.stance_height_control_strategy == 'SLIDER':
    stance_height_axis_id = controller_mapping.stance_height
    controller.add_axis_event_handler(stance_height_axis_id, stance_height_triggers_event, igor, state, deadzone_clip_scaled(igor, 5.0))

  # Like stance height events, wrist velocity events have the same conditional logic
  if controller_mapping.wrist_velocity_control_strategy == 'BUTTONS':
    wrist_vel_lower_btn_id = controller_mapping.wrist_vel[0]
    wrist_vel_raise_btn_id = controller_mapping.wrist_vel[1]
    controller.add_button_event_handler(wrist_vel_lower_btn_id, bind_to_method(wrist_vel_event__buttons, igor, -0.5))
    controller.add_button_event_handler(wrist_vel_raise_btn_id, bind_to_method(wrist_vel_event__buttons, igor, 0.5))
  elif controller_mapping.wrist_velocity_control_strategy == 'SLIDER':
    wrist_vel_axis_id = controller_mapping.wrist_vel
    wrist_vel_event = bind_to_method(wrist_vel_event__axis, igor, deadzone_clip(igor))
    controller.add_axis_event_handler(wrist_vel_axis_id, wrist_vel_event)

  # TODO: soft shutdown button


def register_igor_event_handlers(igor):
  """
  Registers all Igor joystick event handlers
  """
  _add_event_handlers(igor, igor.joystick, igor.config.controller_mapping)
