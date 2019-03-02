from functools import partial as funpart
from util import math_utils


import sdl2


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


def arm_z_vel_raise_event(igor, ts, button_value):
  """
  Event handler to respond to raising arm (z direction) request

  :param igor:         (bound parameter)
  :param ts:           (ignored)
  :param button_value: button pressed state
  """
  if button_value:
    igor.left_arm.set_z_velocity(0.2)
    igor.right_arm.set_z_velocity(0.2)


def zero_arm_vel_z_event(igor, both_shoulders_zeroing, ts, pressed):
  """
  Event handler when either the lower or raise arm button is pressed or released.
  This event handler will zero the Z axis velocity of the arms if both buttons
  are being pressed simultaneously. If this condition is not satisfied,
  then this function does nothing.

  :param igor:                   (bound parameter)
  :param both_shoulders_zeroing: (bound parameter)
  :param ts:                     (ignored)
  :param pressed:                (ignored)
  """
  if both_shoulders_zeroing():
    igor.left_arm.set_z_velocity(0.0)
    igor.right_arm.set_z_velocity(0.0)


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


def bind_both_shoulders_zeroing(joystick, lower_btn_id, raise_btn_id):
  return lambda: joystick.get_button(lower_btn_id) == joystick.get_button(raise_btn_id)


# ------------------------------------------------------------------------------
# Register event handlers
# ------------------------------------------------------------------------------


def _register_sdl_joystick(igor):
  joystick = igor.joystick

  _l1_index = sdl2.SDL_CONTROLLER_BUTTON_LEFTSHOULDER
  _l2_index = sdl2.SDL_CONTROLLER_AXIS_TRIGGERLEFT
  _r1_index = sdl2.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER
  _r2_index = sdl2.SDL_CONTROLLER_AXIS_TRIGGERRIGHT

  # ----------------------------------------------
  # Functions to be passed to event handlers below

  def stance_height_calc():
    l_val = joystick.get_axis(_l2_index) # equivalent to `get_axis('L2')`
    r_val = joystick.get_axis(_r2_index) # equivalent to `get_axis('R2')`
    d_ax = l_val-r_val
    if abs(d_ax) > igor.joystick_dead_zone:
      return 0.5*d_ax
    return 0.0

  # ----------------------------------------------------------------------
  # Functions which have bound parameters, in order to have right function
  # signature for event handlers

  # ------------
  # Quit Session

  quit_session = funpart(quit_session_event, igor)
  joystick.add_button_event_handler('SHARE', quit_session)

  # ---------------
  # Toggle Balancer

  balance_controller = funpart(balance_controller_event, igor)
  joystick.add_button_event_handler('TOUCHPAD', balance_controller)

  # -----------------------
  # Left Arm event handlers

  # Reacts to left stick Y-axis
  arm_x_vel = funpart(arm_x_vel_event, igor, bind_arm_x_deadzone(igor))
  joystick.add_axis_event_handler('LEFT_STICK_Y', arm_x_vel)

  # Reacts to left stick X-axis
  arm_y_vel = funpart(arm_y_vel_event, igor, bind_arm_y_deadzone(igor))
  joystick.add_axis_event_handler('LEFT_STICK_X', arm_y_vel)

  # Reacts to left trigger axis
  arm_z_vel_lt = funpart(arm_z_vel_lower_event, igor, _r1_index)
  joystick.add_button_event_handler('L1', arm_z_vel_lt)

  # Reacts to right trigger axis
  arm_z_vel_rt = funpart(arm_z_vel_raise_event, igor)
  joystick.add_button_event_handler('R1', arm_z_vel_rt)

  # ------------------------
  # Both Arms event handlers

  # Reacts to L1/R1 pressed/released
  zero_arm_z = funpart(zero_arm_vel_z_event, igor, bind_both_shoulders_zeroing(_l1_index, _r1_index))
  joystick.add_button_event_handler('L1', zero_arm_z)
  joystick.add_button_event_handler('R1', zero_arm_z)

  # Reacts to D-Pad pressed/released
  # TODO
  #wrist_vel = funpart(wrist_vel_event, igor)
  #joystick.add_dpad_event_handler(wrist_vel)

  # ----------------------
  # Chassis event handlers

  # Reacts to right stick Y-axis
  chassis_velocity = funpart(chassis_velocity_event, igor, bind_arm_y_deadzone(igor))
  joystick.add_axis_event_handler('RIGHT_STICK_Y', chassis_velocity)

  # Reacts to right stick X-axis
  chassis_yaw = funpart(chassis_yaw_event, igor, bind_arm_y_deadzone(igor))
  joystick.add_axis_event_handler('RIGHT_STICK_X', chassis_yaw)

  # -------------
  # Stance height

  stance_height_trigger = funpart(stance_height_triggers_event, igor, joystick, stance_height_calc, 'OPTIONS')
  joystick.add_axis_event_handler('L2', stance_height_trigger)
  joystick.add_axis_event_handler('R2', stance_height_trigger)

  stance_height = funpart(stance_height_event, igor, stance_height_calc)
  joystick.add_button_event_handler('OPTIONS', stance_height)

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


def _register_mobile_io(igor):
  joystick = igor.joystick
  print('Mobile IO app will drive Igor')
  print('Press b1 to quit')

  ARM_VEL_X_AXIS = 'a2'
  ARM_VEL_Y_AXIS = 'a1'
  STANCE_HEIGHT_AXIS = 'a3'
  WRIST_VEL_AXIS = 'a6'
  CHASSIS_YAW_AXIS = 'a7'
  CHASSIS_VEL_AXIS = 'a8'
  QUIT_BTN = 'b1'
  BALANCE_CONTROLLER_TOGGLE_BTN = 'b2'
  SOFT_SHUTDOWN_BTN = 'b4'
  LOWER_ARM_BTN = 'b8'
  RAISE_ARM_BTN = 'b6'

  # ----------------------------------------------
  # Functions to be passed to event handlers below

  def stance_height_calc():
    val = joystick.get_axis(STANCE_HEIGHT_AXIS)
    d_ax = val
    if abs(d_ax) > igor.joystick_dead_zone:
      return 0.5*d_ax
    return 0.0

  # ----------------------------------------------------------------------
  # Functions which have bound parameters, in order to have right function
  # signature for event handlers

  # ------------
  # Quit Session

  quit_session = funpart(quit_session_event, igor)
  joystick.add_button_event_handler(QUIT_BTN, quit_session)

  # ---------------
  # Toggle Balancer

  balance_controller = funpart(balance_controller_event, igor)
  joystick.add_button_event_handler(BALANCE_CONTROLLER_TOGGLE_BTN, balance_controller)

  # -----------------------
  # Left Arm event handlers

  # Reacts to left stick Y-axis
  arm_x_vel = funpart(arm_x_vel_event, igor, bind_arm_x_deadzone(igor))
  joystick.add_axis_event_handler(ARM_VEL_X_AXIS, arm_x_vel)

  # Reacts to left stick X-axis
  arm_y_vel = funpart(arm_y_vel_event, igor, bind_arm_y_deadzone(igor))
  joystick.add_axis_event_handler(ARM_VEL_Y_AXIS, arm_y_vel)

  # Reacts to left trigger axis
  arm_z_vel_lt = funpart(arm_z_vel_event_l, igor, RAISE_ARM_BTN)
  joystick.add_button_event_handler(LOWER_ARM_BTN, arm_z_vel_lt)

  # Reacts to right trigger axis
  arm_z_vel_rt = funpart(arm_z_vel_event_r, igor)
  joystick.add_button_event_handler(RAISE_ARM_BTN, arm_z_vel_rt)

  # ------------------------
  # Both Arms event handlers

  # Reacts to L1/R1 pressed/released
  zero_arm_z = funpart(zero_arm_vel_z_event, igor, bind_both_shoulders_zeroing(LOWER_ARM_BTN, RAISE_ARM_BTN))
  joystick.add_button_event_handler(LOWER_ARM_BTN, zero_arm_z)
  joystick.add_button_event_handler(RAISE_ARM_BTN, zero_arm_z)

  # Reacts to D-Pad pressed/released
  wrist_vel = funpart(wrist_vel_event, igor, bind_wrist_deadzone(igor))
  joystick.add_axis_event_handler(WRIST_VEL_AXIS, wrist_vel)

  # ----------------------
  # Chassis event handlers

  # Reacts to right stick Y-axis
  chassis_velocity = funpart(chassis_velocity_event, igor, bind_arm_y_deadzone(igor))
  joystick.add_axis_event_handler(CHASSIS_VEL_AXIS, chassis_velocity)

  # Reacts to right stick X-axis
  chassis_yaw = funpart(chassis_yaw_event, igor, bind_arm_y_deadzone(igor))
  joystick.add_axis_event_handler(CHASSIS_YAW_AXIS, chassis_yaw)

  # -------------
  # Stance height

  stance_height_trigger = funpart(stance_height_triggers_event, igor, joystick, stance_height_calc, SOFT_SHUTDOWN_BTN)
  joystick.add_axis_event_handler(STANCE_HEIGHT_AXIS, stance_height_trigger)

  stance_height = funpart(stance_height_event, igor, stance_height_calc)
  joystick.add_button_event_handler(SOFT_SHUTDOWN_BTN, stance_height)


_controller_init_hooks = {
  'GameController': _register_sdl_joystick,
  'MobileIO': _register_mobile_io}


def register_igor_event_handlers(igor):
  """
  Registers all Igor joystick event handlers
  """
  joystick = igor.joystick
  controller_type = joystick.controller_type

  if controller_type not in _controller_init_hooks.keys():
    raise RuntimeError('Cannot find a suitable interface for the given Igor controller {0}'.format(joystick))

  _controller_init_hooks[controller_type](igor)
