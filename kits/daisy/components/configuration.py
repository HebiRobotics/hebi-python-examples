import os


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
        msg = 'Could not find Mobile IO on network with\nfamily: {0}\nname: {1}'.format(family, name)
        print(msg)
        sleep(1)
        mio = hebi.util.create_mobile_io(lookup, family, name)

    return mio  # HebiModuleController(group)


# ------------------------------------------------------------------------------
# Controller Mappings
# ------------------------------------------------------------------------------


class HexapodControllerMapping(object):
    __slots__ = ('_body_height', '_pitch', '_rotate', '_translate_x', '_translate_y', '_quit', '_mode_selection')

    def __init__(self, body_height, pitch, rotate, translate_x, translate_y, quit, mode_selection):
        self._body_height = body_height
        self._pitch = pitch
        self._rotate = rotate
        self._translate_x = translate_x
        self._translate_y = translate_y
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
    def quit(self):
        return self._quit

    @property
    def mode_selection(self):
        return self._mode_selection


_default_mobile_io_mapping = HexapodControllerMapping(body_height='a3', pitch='a2',
                                                      rotate='a1', translate_x='a8',
                                                      translate_y='a7', quit='b8',
                                                      mode_selection='b1')


# ------------------------------------------------------------------------------
# Configuration and parameters
# ------------------------------------------------------------------------------


class Parameters(object):
    """Used to modify the runtime behavior of a Hexapod."""

    __slots__ = (
        "stance_radius",
        "default_body_height",
        "min_z",
        "max_z",
        "max_r",
        "step_threshold_rotate",
        "step_threshold_shift",
        "joystick_dead_zone",
        "resource_directory",
    )

    def __init__(self):
        self.stance_radius = 0.55
        self.default_body_height = 0.21
        self.min_z = -0.3
        self.max_z = -0.05
        self.max_r = 0.18
        self.step_threshold_rotate = 0.05
        self.step_threshold_shift = 0.03
        self.joystick_dead_zone = 0.05  # FIXME: Tweak as needed

        from os.path import abspath, dirname, join

        self.resource_directory = abspath(join(dirname(__file__), '..', 'resources'))


class HexapodConfig(object):
    """Used when starting up a Hexapod."""

    def __init__(self, imitation=False, family='Daisy', robot_feedback_frequency=200.0, robot_command_lifetime=500):
        self._module_names = [
            'L1_J1_base', 'L1_J2_shoulder', 'L1_J3_elbow',
            'L2_J1_base', 'L2_J2_shoulder', 'L2_J3_elbow',
            'L3_J1_base', 'L3_J2_shoulder', 'L3_J3_elbow',
            'L4_J1_base', 'L4_J2_shoulder', 'L4_J3_elbow',
            'L5_J1_base', 'L5_J2_shoulder', 'L5_J3_elbow',
            'L6_J1_base', 'L6_J2_shoulder', 'L6_J3_elbow'
        ]
        self._family = family
        self._resources_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resources'))
        self._imitation = imitation
        self._find_joystick_strategy = None
        self._controller_mapping = None
        self._robot_feedback_frequency = robot_feedback_frequency
        self._robot_command_lifetime = robot_command_lifetime

    def select_controller_by_mobile_io(self, family, name, feedback_frequency=50):
        """Set the joystick selection strategy to select a mobile IO module
        with the provided family and name."""
        self._find_joystick_strategy = lambda: _controller_by_mobile_io_selector(family, name, feedback_frequency)
        self._controller_mapping = _default_mobile_io_mapping

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
    def resources_directory(self):
        return self._resources_directory

    @property
    def setup_controller(self):
        return self._find_joystick_strategy

    @property
    def controller_mapping(self):
        return self._controller_mapping

    @property
    def robot_feedback_frequency(self):
        return self._robot_feedback_frequency

    @property
    def robot_command_lifetime(self):
        return self._robot_command_lifetime
