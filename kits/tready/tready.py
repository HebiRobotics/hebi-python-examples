from .treaded_base_core import TreadedBase, TreadedBaseControl

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.group import Group


class TreadyBase(TreadedBase):
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

    WHEEL_DIAMETER = 0.125  # m
    WHEEL_BASE = 0.285  # m

    WHEEL_RADIUS = WHEEL_DIAMETER / 2

    TORSO_TORQUE_SCALE = 2.5  # Nm
    TORQUE_MODE_MAX = 25  # Nm

    def __init__(
        self, group: "Group", chassis_ramp_time: float, flipper_ramp_time: float
    ):
        super().__init__(group, chassis_ramp_time, flipper_ramp_time)


class TreadyControl(TreadedBaseControl):
    FLIPPER_VEL_SCALE = 1  # rad/sec
    SPEED_MAX_LIN = 0.25  # m/s, for tracks driven by R8-9+

    def __init__(self, base: TreadyBase):
        super().__init__(base)
