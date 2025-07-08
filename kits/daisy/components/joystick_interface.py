_last_toggle_value = 0
import hebi

import typing


if typing.TYPE_CHECKING:
    from .hexapod import Hexapod
    from .configuration import HexapodControllerMapping
    from hebi._internal.mobile_io import MobileIO


# ------------------------------------------------------------------------------
# Register event handlers
# ------------------------------------------------------------------------------


def deadzone_clip_scaled(dead_zone, out_scale, dz_scale=1.0):
    def calculate(val):
        if abs(val) <= dead_zone * dz_scale:
            return 0.0
        else:
            return val * out_scale
    return calculate


def _get_pin_number_from_name(name):
    # HACK: Do this a better way
    return int(name[1:])


def _set_mobile_io_gui_state(controller: 'MobileIO', controller_mapping: 'HexapodControllerMapping'):
    body_height_pin = _get_pin_number_from_name(controller_mapping.body_height_velocity)
    toggle_mode_pin = _get_pin_number_from_name(controller_mapping.mode_selection)
    quit_pin = _get_pin_number_from_name(controller_mapping.quit)

    controller.set_snap(body_height_pin, 0.001)
    controller.set_axis_label(body_height_pin, 'height', blocking=False)
    controller.set_button_mode(toggle_mode_pin, 1)
    controller.set_button_label(toggle_mode_pin, 'Stance', blocking=False)
    controller.set_button_label(quit_pin, 'Quit', blocking=False)


def _add_event_handlers(hexapod: 'Hexapod', controller: 'MobileIO', controller_mapping: 'HexapodControllerMapping'):
    hexapod.add_on_stop_callback(lambda: controller.resetUI())

    _set_mobile_io_gui_state(controller, controller_mapping)

    translation_vel_calc = deadzone_clip_scaled(hexapod.joystick_dead_zone, 0.175)
    rotation_vel_calc = deadzone_clip_scaled(hexapod.joystick_dead_zone, 0.4)

    def feedback_handler(fbk: 'hebi.GroupFeedback'):
        global _last_toggle_value
        pressed = fbk[0].io.b.get_int(int(controller_mapping.mode_selection[-1]))
        if fbk[0].io.b.get_int(int(controller_mapping.quit[-1])) == 1:
            hexapod.request_stop()
        elif pressed != _last_toggle_value:
            hexapod.toggle_mode()
            _last_toggle_value = pressed
        else:
            vel_x = translation_vel_calc(fbk[0].io.a.get_float(int(controller_mapping.translate_x_velocity[-1])))
            vel_y = translation_vel_calc(fbk[0].io.a.get_float(int(controller_mapping.translate_y_velocity[-1])))
            vel_z = translation_vel_calc(fbk[0].io.a.get_float(int(controller_mapping.translate_z_velocity[-1])))

            rot_vel_y = rotation_vel_calc(fbk[0].io.a.get_float(int(controller_mapping.pitch_velocity[-1])))
            rot_vel_z = rotation_vel_calc(fbk[0].io.a.get_float(int(controller_mapping.rotation_velocity[-1])))

            hexapod.set_translation_velocity(vel_x, vel_y, vel_z)

            #hexapod.set_rotation_velocity_y(rot_vel_y)
            hexapod.set_rotation_velocity_z(rot_vel_z)

    controller._group.add_feedback_handler(feedback_handler)


