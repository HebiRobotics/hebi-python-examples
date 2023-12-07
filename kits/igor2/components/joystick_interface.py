import typing
if typing.TYPE_CHECKING:
  from .igor import Igor
  from hebi._internal.mobile_io import MobileIO
  from hebi import GroupFeedback


label_to_pin_map = {
    'a1': lambda fbk: fbk[0].io.a.get_float(1),
    'a2': lambda fbk: fbk[0].io.a.get_float(2),
    'a3': lambda fbk: fbk[0].io.a.get_float(3),
    'a4': lambda fbk: fbk[0].io.a.get_float(4),
    'a5': lambda fbk: fbk[0].io.a.get_float(5),
    'a6': lambda fbk: fbk[0].io.a.get_float(6),
    'a7': lambda fbk: fbk[0].io.a.get_float(7),
    'a8': lambda fbk: fbk[0].io.a.get_float(8),

    'b1': lambda fbk: fbk[0].io.b.get_int(1),
    'b2': lambda fbk: fbk[0].io.b.get_int(2),
    'b3': lambda fbk: fbk[0].io.b.get_int(3),
    'b4': lambda fbk: fbk[0].io.b.get_int(4),
    'b5': lambda fbk: fbk[0].io.b.get_int(5),
    'b6': lambda fbk: fbk[0].io.b.get_int(6),
    'b7': lambda fbk: fbk[0].io.b.get_int(7),
    'b8': lambda fbk: fbk[0].io.b.get_int(8),
}


# ------------------------------------------------------------------------------
# Binding event handler parameters
# ------------------------------------------------------------------------------


def deadzone_clip(igor: 'Igor'):
  def calculate(val):
    if abs(val) <= igor._joy_dead_zone:
      return 0.0
    else:
      return val
  return calculate


def deadzone_clip_scaled(igor: 'Igor', dz_scale, out_scale):
  def calculate(val):
    if abs(val) <= igor._joy_dead_zone * dz_scale:
      return 0.0
    else:
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


def _add_event_handlers(igor: 'Igor', controller: 'MobileIO', controller_mapping):

  arm_vel_calc = deadzone_clip_scaled(igor, 1.0, 0.5)
  chassis_vel_calc = deadzone_clip_scaled(igor, 1.0, 0.4)
  chassis_yaw_calc = deadzone_clip_scaled(igor, 1.0, 25.0 * igor.wheel_radius / igor.wheel_base)
  igor_state = IgorControlState()


  def fbk_handler(fbk: 'GroupFeedback'):
    if label_to_pin_map[controller_mapping.quit](fbk) == 1 and igor.started:
      igor.request_stop()

    igor.set_balance_controller_state(not bool(label_to_pin_map[controller_mapping.balance_controller_toggle](fbk)))

    vel = arm_vel_calc(label_to_pin_map[controller_mapping.arm_vel_x](fbk))
    igor.left_arm.set_x_velocity(vel)
    igor.right_arm.set_x_velocity(vel)
    vel = arm_vel_calc(label_to_pin_map[controller_mapping.arm_vel_y](fbk))
    igor.left_arm.set_y_velocity(-vel)
    igor.right_arm.set_y_velocity(-vel)

    raise_button_value = label_to_pin_map[controller_mapping.raise_arm](fbk)
    lower_button_value = label_to_pin_map[controller_mapping.lower_arm](fbk)
    if raise_button_value == 1 and lower_button_value == 1:
      pass
    elif lower_button_value:
      igor.left_arm.set_z_velocity(-0.2)
      igor.right_arm.set_z_velocity(-0.2)
    elif raise_button_value:
      igor.left_arm.set_z_velocity(0.2)
      igor.right_arm.set_z_velocity(0.2)
    else:
      igor.left_arm.set_z_velocity(0.0)
      igor.right_arm.set_z_velocity(0.0)

    igor.chassis.set_directional_velocity(chassis_vel_calc(label_to_pin_map[controller_mapping.chassis_vel](fbk)))
    igor.chassis.set_yaw_velocity(chassis_yaw_calc(label_to_pin_map[controller_mapping.chassis_yaw](fbk)))
    igor.chassis.set_i_term_adjustment(label_to_pin_map[controller_mapping.i_term_adjust](fbk))

    if label_to_pin_map[controller_mapping.soft_shutdown](fbk) == 1:
      igor_state.soft_shutdown_enabled = True
      # The Leg class has a software position limit which will set actual velocity to 0 if getting near the safety limits
      igor.left_leg.set_knee_velocity(1.0)
      igor.right_leg.set_knee_velocity(1.0)
    else:
      igor_state.soft_shutdown_enabled = False

    igor.allow_transition_to_idle(igor_state.soft_shutdown_enabled)

    # Only do this if soft shutdown is currently being requested
    if not igor_state.soft_shutdown_enabled:
      val = deadzone_clip_scaled(igor, 5.0, -1.0)(label_to_pin_map[controller_mapping.stance_height](fbk))
      igor.left_leg.set_knee_velocity(val)
      igor.right_leg.set_knee_velocity(val)

    vel = deadzone_clip(igor)(label_to_pin_map[controller_mapping.wrist_vel](fbk))
    igor.left_arm.set_wrist_velocity(vel)
    igor.right_arm.set_wrist_velocity(vel)

  controller._group.add_feedback_handler(fbk_handler)


def register_igor_event_handlers(igor: 'Igor'):
  """
  Registers all Igor joystick event handlers
  """
  _add_event_handlers(igor, igor._mobile_io, igor._config.controller_mapping)
