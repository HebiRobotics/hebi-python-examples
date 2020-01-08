_last_toggle_value = False
import hebi


# ------------------------------------------------------------------------------
# Event handlers
# ------------------------------------------------------------------------------


def rotation_y_vel_event(hexapod, vel_calc, ts, axis_value):
  """
  This event handler will set the Hexapod's horizontal rotation velocity a value
  proportional to the given input `axis_value` when outside the deadzone. When
  the value is within the deadzone, the horizontal rotation velocity is set to zero.

  :param hexapod:          (bound parameter)
  :param vel_calc:         (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  vel = vel_calc(axis_value)
  # Note: C++ demo does not have rotate y handler.
  #hexapod.set_rotation_velocity_y(vel)


def rotation_z_vel_event(hexapod, vel_calc, ts, axis_value):
  """
  This event handler will set the Hexapod's vertical rotation velocity a value
  proportional to the given input `axis_value` when outside the deadzone. When
  the value is within the deadzone, the vertical rotation velocity is set to zero.

  :param hexapod:          (bound parameter)
  :param vel_calc:         (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  vel = vel_calc(axis_value)
  hexapod.set_rotation_velocity_z(vel)


def translation_x_vel_event(hexapod, vel_calc, ts, axis_value):
  """
  This event handler will set the Hexapod's forward/backward translation velocity a value
  proportional to the given input `axis_value` when outside the deadzone. When
  the value is within the deadzone, the forward/backward translation velocity is set to zero.

  :param hexapod:          (bound parameter)
  :param vel_calc:         (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  vel = vel_calc(axis_value)
  hexapod.set_translation_velocity_x(-vel)


def translation_y_vel_event(hexapod, vel_calc, ts, axis_value):
  """
  This event handler will set the Hexapod's horizontal translation velocity a value
  proportional to the given input `axis_value` when outside the deadzone. When
  the value is within the deadzone, the horizontal translation velocity is set to zero.

  :param hexapod:          (bound parameter)
  :param vel_calc:         (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  vel = vel_calc(axis_value)
  hexapod.set_translation_velocity_y(vel)


def translation_z_vel_event(hexapod, vel_calc, ts, axis_value):
  """
  This event handler will set the Hexapod's vertical translation velocity a value
  proportional to the given input `axis_value` when outside the deadzone. When
  the value is within the deadzone, the vertical translation velocity is set to zero.

  :param hexapod:          (bound parameter)
  :param vel_calc:         (bound parameter)
  :param ts:               (ignored)
  :param axis_value:       [-1,1] value of the axis
  """
  vel = vel_calc(axis_value)
  hexapod.set_translation_velocity_z(-vel)


def toggle_mode_event(hexapod, ts, pressed):
  """
  Event handler to react to the "toggle" button being pressed or released.
  When the button is pressed, and the robot is ready (in the "started" state),
  the mode will toggle.
  """
  global _last_toggle_value
  if _last_toggle_value == pressed:
    # Ignore, since we care about edge triggers
    return

  _last_toggle_value = pressed
  if pressed and hexapod.started:
    hexapod.update_mode(None)


def quit_session_event(hexapod, ts, pressed):
  """
  Event handler to react to the "toggle" button being pressed or released.
  When the button is pressed, and the robot is ready (in the "started" state),
  the session will begin to quit.
  """
  if pressed and hexapod.started:
    hexapod.request_stop()


# ------------------------------------------------------------------------------
# Register event handlers
# ------------------------------------------------------------------------------


def deadzone_clip_scaled(hexapod, out_scale, dz_scale=1.0):
  def calculate(val):
    if abs(val) <= hexapod.joystick_dead_zone * dz_scale:
      return 0.0
    else:
      return val * out_scale
  return calculate


def _get_pin_number_from_name(name):
  # HACK: Do this a better way
  return int(name[1:])


def _set_mobile_io_gui_state(hexapod, controller_mapping):
  cmd = hebi.GroupCommand(1)

  io = cmd.io
  a_bank = io.a
  b_bank = io.b
  e_bank = io.e

  body_height_pin = _get_pin_number_from_name(controller_mapping.body_height_velocity)
  toggle_mode_pin = _get_pin_number_from_name(controller_mapping.mode_selection)
  quit_pin = _get_pin_number_from_name(controller_mapping.quit)

  a_bank.set_float(body_height_pin, 0.001) # Use a small value as opposed to 0.0 to work around bug in older versions of Mobile IO app
  b_bank.set_int(toggle_mode_pin, 1) # Set to toggle mode
  e_bank.set_int(toggle_mode_pin, 1) # Highlight
  e_bank.set_int(quit_pin, 1)        # Highlight

  hexapod.joystick.group.send_command(cmd)


def _add_event_handlers(hexapod, controller, controller_mapping):
  # Shortcut closure to get around having to use functool.partial
  def bind_to_method(method, *args):
    def bound_meth(ts, val):
      method(*args, ts, val)
    return bound_meth

  def reset_mobile_io_gui_state():
    cmd = hebi.GroupCommand(1)

    io = cmd.io
    a_bank = io.a
    b_bank = io.b
    e_bank = io.e
    # set UI back to original state
    body_height_pin = _get_pin_number_from_name(controller_mapping.body_height_velocity)
    toggle_mode_pin = _get_pin_number_from_name(controller_mapping.mode_selection)
    quit_pin = _get_pin_number_from_name(controller_mapping.quit)

    a_bank.set_float(body_height_pin, None)
    b_bank.set_int(toggle_mode_pin, 0)
    e_bank.set_int(toggle_mode_pin, 0)
    e_bank.set_int(quit_pin, 0)

    hexapod.joystick.group.send_command(cmd)

  hexapod.add_on_stop_callback(reset_mobile_io_gui_state)

  _set_mobile_io_gui_state(hexapod, controller_mapping)

  translation_vel_calc = deadzone_clip_scaled(hexapod, 0.175)
  rotation_vel_calc = deadzone_clip_scaled(hexapod, 0.4)

  # Quit event
  controller.add_button_event_handler(controller_mapping.quit, bind_to_method(quit_session_event, hexapod))

  # Toggle mode event
  controller.add_button_event_handler(controller_mapping.mode_selection, bind_to_method(toggle_mode_event, hexapod))

  # Rotation events
  # Note: Might need to swap y and z vel event handlers here
  controller.add_axis_event_handler(controller_mapping.pitch_velocity, bind_to_method(rotation_y_vel_event, hexapod, rotation_vel_calc))
  controller.add_axis_event_handler(controller_mapping.rotation_velocity, bind_to_method(rotation_z_vel_event, hexapod, rotation_vel_calc))
  
  # Velocity translation events
  controller.add_axis_event_handler(controller_mapping.translate_x_velocity, bind_to_method(translation_x_vel_event, hexapod, translation_vel_calc))
  controller.add_axis_event_handler(controller_mapping.translate_y_velocity, bind_to_method(translation_y_vel_event, hexapod, translation_vel_calc))
  controller.add_axis_event_handler(controller_mapping.body_height_velocity, bind_to_method(translation_z_vel_event, hexapod, translation_vel_calc))


def register_hexapod_event_handlers(hexapod):
  _add_event_handlers(hexapod, hexapod.joystick, hexapod.config.controller_mapping)
