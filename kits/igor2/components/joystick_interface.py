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


def wrist_vel_event(igor, in_deadzone_func, ts, value):
  """
  This event handler will set the velocity of the wrists.

  :param igor:              (bound parameter)
  :param in_deadzone_func:  (bound parameter)
  :param ts:                (ignored)
  :param value:             [-1,1] value
  """
  l_arm = igor.left_arm
  r_arm = igor.right_arm
  if in_deadzone_func(value):
    l_arm.set_wrist_velocity(0.0)
    r_arm.set_wrist_velocity(0.0)
  else:
    joy_low_pass = 0.95
    vel = (joy_low_pass*l_arm.user_commanded_wrist_velocity)+value*(1.0-joy_low_pass)*0.25
    l_arm.set_wrist_velocity(vel)
    r_arm.set_wrist_velocity(vel)


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


def stance_height_triggers_event(igor, joy, vel_calc, soft_shutdown_btn_id, ts, axis_value):
  """
  Event handler to leg stance height modification request

  :param igor:
  :param joy:
  :param vel_calc:
  :param ts:          (ignored)
  :param axis_value:
  """
  # Ignore this if soft shutdown is currently being requested
  if joy.get_button(soft_shutdown_btn_id):
    return

  val = vel_calc()
  igor.left_leg.set_knee_velocity(val)
  igor.right_leg.set_knee_velocity(val)


def stance_height_event(igor, vel_calc, ts, pressed):
  """
  Event handler to soft shutdown request

  :param igor:     (bound parameter)
  :param vel_calc: (bound parameter)
  :param ts:       (ignored)
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


def bind_wrist_deadzone(igor):
  return lambda val: abs(val) <= igor.joystick_dead_zone


# ------------------------------------------------------------------------------
# Register event handlers
# ------------------------------------------------------------------------------


def _add_event_handlers(igor, controller, controller_mapping):

  # Shortcut closures to get around having to use functool.partial
  def bind_1_to_method(method, arg1):
    def bound_meth(ts, val):
      method(arg1, ts, val)
    return bound_meth

  def bind_2_to_method(method, arg1, arg2):
    def bound_meth(ts, val):
      method(arg1, arg2, ts, val)
    return bound_meth

  arm_x_deadzone_func = bind_arm_x_deadzone(igor)
  arm_y_deadzone_func = bind_arm_y_deadzone(igor)

  controller.add_button_event_handler(controller_mapping.quit, bind_1_to_method(quit_session_event, igor))
  controller.add_button_event_handler(controller_mapping.balance_controller_toggle, bind_1_to_method(balance_controller_event, igor))

  controller.add_axis_event_handler(controller_mapping.arm_vel_x, bind_2_to_method(arm_x_vel_event, igor, arm_x_deadzone_func))
  controller.add_axis_event_handler(controller_mapping.arm_vel_y, bind_2_to_method(arm_y_vel_event, igor, arm_y_deadzone_func))
  controller.add_axis_event_handler(controller_mapping.lower_arm, bind_2_to_method(arm_z_vel_lower_event, igor, controller_mapping.raise_arm))
  controller.add_axis_event_handler(controller_mapping.raise_arm, bind_2_to_method(arm_z_vel_raise_event, igor, controller_mapping.lower_arm))

  controller.add_axis_event_handler(controller_mapping.chassis_vel, bind_2_to_method(chassis_velocity_event, igor, arm_y_deadzone_func))
  controller.add_axis_event_handler(controller_mapping.chassis_yaw, bind_2_to_method(chassis_yaw_event, igor, arm_y_deadzone_func))

  # Stance height events are responded to differently by different kinds of controllers, thus requiring a bit more complex logic
  if controller_mapping.stance_height_control_strategy == 'TRIGGERS':
    stance_height_lower_axis_id = controller_mapping.stance_height[0]
    stance_height_raise_axis_id = controller_mapping.stance_height[1]
    #controller.add_axis_event_handler(stance_height_lower_axis_id, ...)
    #controller.add_axis_event_handler(stance_height_raise_axis_id, ...)
  elif controller_mapping.stance_height_control_strategy == 'SLIDER':
    stance_height_axis_id = controller_mapping.stance_height
    #controller.add_axis_event_handler(stance_height_axis_id, ...)

  # Like stance height events, wrist velocity events have the same conditional logic
  if controller_mapping.wrist_velocity_control_strategy == 'BUTTONS':
    wrist_vel_lower_btn_id = controller_mapping.wrist_vel[0]
    wrist_vel_raise_btn_id = controller_mapping.wrist_vel[1]
    #controller.add_button_event_handler(wrist_vel_lower_btn_id, ...)
    #controller.add_button_event_handler(wrist_vel_raise_btn_id, ...)
  elif controller_mapping.wrist_velocity_control_strategy == 'SLIDER':
    wrist_vel_axis_id = controller_mapping.wrist_vel
    # Note: additionally use `bind_wrist_deadzone` for deadzone calculation
    #controller.add_axis_event_handler(wrist_vel_axis_id, ...)


def register_igor_event_handlers(igor):
  """
  Registers all Igor joystick event handlers
  """
  _add_event_handlers(igor, igor.joystick, igor.config.controller_mapping)
