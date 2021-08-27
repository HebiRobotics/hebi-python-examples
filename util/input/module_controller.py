import hebi


class FeedbackData(object):
    """Data to be passed to subscribed handlers."""
    __slots__ = ('_a1', '_a2', '_a3', '_a4', '_a5', '_a6', '_a7', '_a8',
                 '_b1', '_b2', '_b3', '_b4', '_b5', '_b6', '_b7', '_b8', '_time')

    def __init__(self):
        self._a1 = 0.0
        self._a2 = 0.0
        self._a3 = 0.0
        self._a4 = 0.0
        self._a5 = 0.0
        self._a6 = 0.0
        self._a7 = 0.0
        self._a8 = 0.0
        self._b1 = 0
        self._b2 = 0
        self._b3 = 0
        self._b4 = 0
        self._b5 = 0
        self._b6 = 0
        self._b7 = 0
        self._b8 = 0
        self._time = 0

    @property
    def a1(self):
        return self._a1

    @property
    def a2(self):
        return self._a2

    @property
    def a3(self):
        return self._a3

    @property
    def a4(self):
        return self._a4

    @property
    def a5(self):
        return self._a5

    @property
    def a6(self):
        return self._a6

    @property
    def a7(self):
        return self._a7

    @property
    def a8(self):
        return self._a8

    @property
    def b1(self):
        return self._b1

    @property
    def b2(self):
        return self._b2

    @property
    def b3(self):
        return self._b3

    @property
    def b4(self):
        return self._b4

    @property
    def b5(self):
        return self._b5

    @property
    def b6(self):
        return self._b6

    @property
    def b7(self):
        return self._b7

    @property
    def b8(self):
        return self._b8

    @property
    def time(self):
        return self._time


_getter_dict = {
    'a1': lambda data: data.a1,
    'a2': lambda data: data.a2,
    'a3': lambda data: data.a3,
    'a4': lambda data: data.a4,
    'a5': lambda data: data.a5,
    'a6': lambda data: data.a6,
    'a7': lambda data: data.a7,
    'a8': lambda data: data.a8,
    'b1': lambda data: data.b1,
    'b2': lambda data: data.b2,
    'b3': lambda data: data.b3,
    'b4': lambda data: data.b4,
    'b5': lambda data: data.b5,
    'b6': lambda data: data.b6,
    'b7': lambda data: data.b7,
    'b8': lambda data: data.b8}


_axis_set = ('a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7', 'a8')
_button_set = ('b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'b7', 'b8')


def _transformer_for_field(field):
    return _getter_dict[field]


def _fill_feedback_data(data_dst, hebi_feedback_src):
    io_a = hebi_feedback_src.io.a
    io_b = hebi_feedback_src.io.b
    data_dst._a1 = io_a.get_float(1)[0]
    data_dst._a2 = io_a.get_float(2)[0]
    data_dst._a3 = io_a.get_float(3)[0]
    data_dst._a4 = io_a.get_float(4)[0]
    data_dst._a5 = io_a.get_float(5)[0]
    data_dst._a6 = io_a.get_float(6)[0]
    data_dst._a7 = io_a.get_float(7)[0]
    data_dst._a8 = io_a.get_float(8)[0]
    data_dst._b1 = io_b.get_int(1)[0]
    data_dst._b2 = io_b.get_int(2)[0]
    data_dst._b3 = io_b.get_int(3)[0]
    data_dst._b4 = io_b.get_int(4)[0]
    data_dst._b5 = io_b.get_int(5)[0]
    data_dst._b6 = io_b.get_int(6)[0]
    data_dst._b7 = io_b.get_int(7)[0]
    data_dst._b8 = io_b.get_int(8)[0]
    data_dst._time = hebi_feedback_src.receive_time_us[0]


class HebiModuleController(object):
    """Provides the same interface as a Joystick but backed by a physical HEBI
    group.

    The device in the group must be able to provide IO data from banks A
    and B. The preferred use of this class is to be backed by an
    iOS/Android enabled device using the HEBI Mobile IO app, but a HEBI
    IO board in theory can work as well.
    """
    __slots__ = ['_group', '_event_handlers', '_feedback_data', '_current_feedback', "_current_feedback_lock"]

    def __init__(self, group):
        if group.size != 1:
            raise RuntimeError('Group must be of a single module')

        self._group = group
        group.add_feedback_handler(self.__fbk_handler)
        self._event_handlers = list()
        self._feedback_data = FeedbackData()
        self._current_feedback = hebi.GroupFeedback(group.size)
        from threading import Lock
        self._current_feedback_lock = Lock()

    def __fbk_handler(self, feedback):
        # Check this to avoid having to update the feedback
        if len(self._event_handlers) == 0:
            return

        with self._current_feedback_lock:
            self._current_feedback.copy_from(feedback)

        data = self._feedback_data
        _fill_feedback_data(data, feedback)
        for entry in self._event_handlers:
            entry(data)

    def add_axis_event_handler(self, axis, handler):
        if axis not in _axis_set:
            raise ValueError('{0} is not a valid axis'.format(axis))

        cvtr = _getter_dict[axis]

        def wrapped_handler(data):
            handler(data.time, cvtr(data))
        self._event_handlers.append(wrapped_handler)

    def add_button_event_handler(self, button, handler):
        if button not in _button_set:
            raise ValueError('{0} is not a valid button'.format(button))

        cvtr = _getter_dict[button]

        def wrapped_handler(data):
            handler(data.time, cvtr(data))
        self._event_handlers.append(wrapped_handler)

    def get_axis(self, axis):
        if axis not in _axis_set:
            raise ValueError('{0} is not a valid axis'.format(axis))
        return _getter_dict[axis](self._feedback_data)

    def get_slider(self, slider):
        """Alias for :meth:`.get_axis`."""
        if slider not in _axis_set:
            raise ValueError('{0} is not a valid slider'.format(slider))
        return _getter_dict[slider](self._feedback_data)

    def get_button(self, button):
        if button not in _button_set:
            raise ValueError('{0} is not a valid button'.format(button))
        return _getter_dict[button](self._feedback_data)

    @property
    def controller_type(self):
        return 'MobileIO'

    @property
    def feedback_frequency(self):
        return self._group.feedback_frequency

    @feedback_frequency.setter
    def feedback_frequency(self, value):
        """Set the feedback frequency of the group.

        This will be the frequency at which your IO data is updated.
        """
        self._group.feedback_frequency = value

    @property
    def group(self):
        return self._group

    @property
    def current_feedback(self):
        """The last feedback received from the module."""
        return self._current_feedback
