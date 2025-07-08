
from dataclasses import dataclass

import os
from os.path import abspath, dirname, join
from hebi.config import HebiConfig


# ------------------------------------------------------------------------------
# Controller selectors
# ------------------------------------------------------------------------------


def _controller_by_mobile_io_selector(family, name, feedback_frequency):
    import hebi
    from time import sleep
    lookup = hebi.Lookup()
    sleep(2)
    mio = hebi.util.create_mobile_io(lookup, family, name)
    while mio is None:
        msg = f'Could not find Mobile IO on network with\nfamily: {family}\nname: {name}'
        print(msg)
        sleep(1)
        mio = hebi.util.create_mobile_io(lookup, family, name)

    return mio  # HebiModuleController(group)


# ------------------------------------------------------------------------------
# Controller Mappings
# ------------------------------------------------------------------------------


class HexapodControllerMapping:
    __slots__ = ('_body_height', '_pitch', '_rotate', '_translate_x', '_translate_y', '_translate_z', '_quit', '_mode_selection')

    def __init__(self, body_height, pitch, rotate, translate_x, translate_y, translate_z, quit, mode_selection):
        self._body_height = body_height
        self._pitch = pitch
        self._rotate = rotate
        self._translate_x = translate_x
        self._translate_y = translate_y
        self._translate_z = translate_z
        self._quit = quit
        self._mode_selection = mode_selection

    @property
    def body_height_velocity(self):
        return self._body_height

    @property
    def pitch_velocity(self):
        return self._pitch

    @property
    def rotation_velocity(self):
        return self._rotate

    @property
    def translate_x_velocity(self):
        """Forward/Backwards from driver's perspective."""
        return self._translate_x

    @property
    def translate_y_velocity(self):
        """Horizontal from driver's perspective."""
        return self._translate_y

    @property
    def translate_z_velocity(self):
        """Vertical from driver's perspective."""
        return self._translate_z

    @property
    def quit(self):
        return self._quit

    @property
    def mode_selection(self):
        return self._mode_selection


# ------------------------------------------------------------------------------
# Configuration and parameters
# ------------------------------------------------------------------------------


@dataclass(slots=True, frozen=True)
class Parameters:
    """Used to modify the runtime behavior of a Hexapod."""

    stance_radius = 0.55
    default_body_height = 0.21
    min_z = -0.3
    max_z = -0.05
    max_r = 0.18
    step_threshold_rotate = 0.05
    step_threshold_shift = 0.03
    joystick_dead_zone = 0.05  # FIXME: Tweak as needed
    resource_directory = abspath(join(dirname(__file__), '..', 'resources'))


class HexapodConfig:
    """Used when starting up a Hexapod."""

    def __init__(self, cfg: HebiConfig):
        
        self._module_names = cfg.names
        self._family = cfg.families[0]

        # defaults
        user_data = cfg.user_data if cfg.user_data is not None else {}
        self._imitation = user_data.get('imitation', False)
        self._robot_feedback_frequency = user_data.get('robot_feedback_frequency', 200.0)
        self._robot_command_lifetime = user_data.get('robot_command_lifetime', 500)

        self._find_joystick_strategy = None
        self._controller_mapping = None
    def select_controller_by_mobile_io(self, mio: 'MobileIO', mapping: 'HexapodControllerMapping'):
        """Set the joystick selection strategy to select a mobile IO module
        with the provided family and name."""
        self._controller_mapping = mapping
        self._controller = mio

    @property
    def module_names(self):
        return self._module_names

    @property
    def family(self):
        return self._family

    @property
    def is_imitation(self):
        return self._imitation

    @property
    def controller(self):
        return self._controller

    @property
    def controller_mapping(self):
        return self._controller_mapping

    @property
    def robot_feedback_frequency(self):
        return self._robot_feedback_frequency

    @property
    def robot_command_lifetime(self):
        return self._robot_command_lifetime
