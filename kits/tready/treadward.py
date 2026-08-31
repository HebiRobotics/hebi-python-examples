from .treaded_base_core import TreadedBase, TreadedBaseControl

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.group import Group


class TreadwardBase(TreadedBase):
    # FRAME CONVENTION:
    # ORIGIN = MID-POINT BETWEEN THE WHEELS
    # +X-AXIS = FORWARD
    # +Y-AXIS = LEFT
    # +Z-AXIS = UP

    #   Left  |   Right
    #   1     |    2
    #         |
    #         |
    #   3     |    4

    WHEEL_DIAMETER = 0.175  # m
    WHEEL_BASE = 0.650  # m

    WHEEL_RADIUS = WHEEL_DIAMETER / 2

    TORSO_TORQUE_SCALE = 1  # Nm
    TORQUE_MODE_MAX = 200  # Nm

    def __init__(
        self, group: "Group", chassis_ramp_time: float, flipper_ramp_time: float
    ):
        super().__init__(group, chassis_ramp_time, flipper_ramp_time)


class TreadwardControl(TreadedBaseControl):
    FLIPPER_VEL_SCALE = 1  # rad/sec
    SPEED_MAX_LIN = 0.60  # m/s, for treadward

    def __init__(self, base: TreadwardBase):
        super().__init__(base)
