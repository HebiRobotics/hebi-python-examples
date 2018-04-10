from functools import partial as funpart


# ------------------------------------------------------------------------------
# Arm Event Handlers
# ------------------------------------------------------------------------------

def arm_x_vel_event(arm, in_deadzone_func, ts, axis_value):
  """
  Event handler when left stick Y-Axis motion occurs.
  This event handler will set the given arm's velocity in the X axis to
  a value proportional to the given input `axis_value` when outside the
  joystick deadzone. When the value is within the joystick deadzone, the
  arm's velocity in the X axis is set to zero.

  (Left Stick Y-Axis event handler)

  :param arm:              (bound parameter)
  :param in_deadzone_func: (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  if in_deadzone_func(axis_value):
    arm.set_x_velocity(0.0)
  else:
    scale = -0.4 # TODO: make into customizable parameter
    axis_value = scale*axis_value
    arm.set_x_velocity(scale*axis_value)


def arm_y_vel_event(arm, in_deadzone_func, ts, axis_value):
  """
  Event handler when left stick X-Axis motion occurs.
  This event handler will set the given arm's velocity in the Y axis to
  a value proportional to the given input `axis_value` when outside the
  joystick deadzone. When the value is within the joystick deadzone, the
  arm's velocity in the Y axis is set to zero.

  (Left Stick X-Axis event handler)

  :param arm:              (bound parameter)
  :param in_deadzone_func: (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  if in_deadzone_func(axis_value):
    arm.set_y_velocity(0.0)
  else:
    scale = -0.4 # TODO: make into customizable parameter
    axis_value = scale*axis_value
    arm.set_y_velocity(axis_value)


def arm_z_vel_event_l(arm, ts, axis_value):
  """
  Event handler when left trigger of joystick has its axis value changed
  (Left Trigger Axis event handler)

  :param arm:        (bound parameter)
  :param ts:         (ignored)
  :param axis_value: [-1,1] value of the axis
  """
  # Left Trigger
  # map [-1,1] to [0,1]
  axis_value = -0.2*(axis_value*0.5 + 0.5)
  set_arm_vel_z(arm, axis_value)


def arm_z_vel_event_r(arm, ts, axis_value):
  """
  Event handler when right trigger of joystick has its axis value changed
  (Right Trigger Axis event handler)

  :param arm:        (bound parameter)
  :param ts:         (ignored)
  :param axis_value: [-1,1] value of the axis
  """
  # Right Trigger
  # map [-1,1] to [0,1]
  axis_value = 0.2*(axis_value*0.5 + 0.5)
  set_arm_vel_z(arm, axis_value)


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


# ------------------------------------------------------------------------------
# Chassis Event Handlers
# ------------------------------------------------------------------------------


def chassis_velocity_event(chassis, ts, axis_value):
  """
  Event handler when right stick Y-Axis motion occurs.
  TODO: document

  :param chassis:      (bound parameter)
  :param ts:           (ignored)
  :param axis_value:   [-1,1] value of the axis
  """
  pass


def chassis_yaw_event(chassis, ts, axis_value):
  """
  Event handler when right stick X-Axis motion occurs.
  TODO: document

  :param chassis:      (bound parameter)
  :param ts:           (ignored)
  :param axis_value:   [-1,1] value of the axis
  """
  pass


# ------------------------------------------------------------------------------
# Misc Event Handlers
# ------------------------------------------------------------------------------


def stance_height_event(igor, ts, pressed):
  """
  Event handler when `OPTIONS` button is pressed or released.
  TODO: Finish documenting

  :param igor:    (bound parameter)
  :param ts:
  :param pressed:
  """
  if pressed:
    # TODO: Soft shutdown procedure
    pass
  else:
    # TODO: Normal stance height control
    pass


def balance_controller_event(igor, ts, pressed):
  """
  Event handler when `TOUCHPAD` button is pressed or released.
  TODO: Finish documenting

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


def register_igor_event_handlers(igor, joystick):
  """
  Registers all Igor joystick event handlers
  """

  # ----------------------------------------------
  # Functions to be passed to event handlers below

  arm_x_deadzone = lambda val: abs(val) <= igor.joystick_dead_zone*3.0
  arm_y_deadzone = lambda val: abs(val) <= igor.joystick_dead_zone

  # The current joystick used is not a global state, so we need to wrap it here
  both_triggers = lambda: both_triggers_released(joystick)

  # ----------------------------------------------------------------------
  # Functions which have bound parameters, in order to have right function
  # signature for event handlers

  # -----------------------
  # Left Arm event handlers

  # Reacts to left stick Y-axis
  l_arm_x_vel = funpart(arm_x_vel_event, igor.left_arm, arm_x_deadzone)
  joystick.add_axis_event_handler('LEFT_STICK_Y', l_arm_x_vel)

  # Reacts to left stick X-axis
  l_arm_y_vel = funpart(arm_y_vel_event, igor.left_arm, arm_y_deadzone)
  joystick.add_axis_event_handler('LEFT_STICK_X', l_arm_y_vel)

  # Reacts to left trigger axis
  l_arm_z_vel_lt = funpart(arm_z_vel_event_l, igor.left_arm)
  joystick.add_axis_event_handler('LEFT_TRIGGER', l_arm_z_vel_lt)

  # Reacts to right trigger axis
  l_arm_z_vel_rt = funpart(arm_z_vel_event_r, igor.left_arm)
  joystick.add_axis_event_handler('RIGHT_TRIGGER', l_arm_z_vel_rt)

  # ------------------------
  # Right Arm event handlers

  # Reacts to left stick Y-axis
  r_arm_x_vel = funpart(arm_x_vel_event, igor.right_arm, arm_x_deadzone)
  joystick.add_axis_event_handler('LEFT_STICK_Y', r_arm_x_vel)

  # Reacts to left stick X-axis
  r_arm_y_vel = funpart(arm_y_vel_event, igor.right_arm, arm_y_deadzone)
  joystick.add_axis_event_handler('LEFT_STICK_X', r_arm_y_vel)

  # Reacts to left trigger axis
  r_arm_z_vel_lt = funpart(arm_z_vel_event_l, igor.right_arm)
  joystick.add_axis_event_handler('LEFT_TRIGGER', r_arm_z_vel_lt)

  # Reacts to right trigger axis
  r_arm_z_vel_rt = funpart(arm_z_vel_event_r, igor.right_arm)
  joystick.add_axis_event_handler('RIGHT_TRIGGER', r_arm_z_vel_rt)

  # ------------------------
  # Both Arms event handlers

  # Reacts to triggers pressed/released
  zero_arm_z = funpart(zero_arm_z_event, igor, both_triggers)
  joystick.add_button_event_handler('LEFT_TRIGGER', zero_arm_z)
  joystick.add_button_event_handler('RIGHT_TRIGGER', zero_arm_z)

  # ----------------------
  # Chassis event handlers

  # Reacts to right stick Y-axis
  chassis_velocity = funpart(chassis_velocity_event, igor.chassis)
  joystick.add_axis_event_handler('RIGHT_STICK_Y', chassis_velocity)

  # Reacts to right stick X-axis
  chassis_yaw = funpart(chassis_yaw_event, igor.chassis)
  joystick.add_axis_event_handler('RIGHT_STICK_X', chassis_yawz)

  # -------------
  # Stance height

  stance_height = funpart(stance_height_event, igor)
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
    # TODO
    def c_set_cam(ts, pressed):
      print('camera position ---> 1.4')
    def sq_set_cam(ts, pressed):
      print('camera position ---> 0.0')
    def t_set_cam(ts, pressed):
      print('camera velocity ---> -0.6')
    def x_set_cam(ts, pressed):
      print('camera velocity ---> 0.6')

    joystick.add_button_event_handler('CIRCLE', c_set_cam)
    joystick.add_button_event_handler('SQUARE', sq_set_cam)
    joystick.add_button_event_handler('TRIANGLE', t_set_cam)
    joystick.add_button_event_handler('X', x_set_cam)

