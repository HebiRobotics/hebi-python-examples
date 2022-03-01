
import hebi
from util.math_utils import quat2rot

import typing
if typing.TYPE_CHECKING:
    from hebi._internal.group import Group


class HebiCamera:
    def __init__(self, group: 'Group'):
        self.group = group
        self.fbk = self.group.get_next_feedback()
        while self.fbk is None:
            self.fbk = group.get_next_feedback()
        self.cmd = hebi.GroupCommand(self.group.size)

    def update(self):
        self.group.get_next_feedback(reuse_fbk=self.fbk)

    def send(self):
        self.group.send_command(self.cmd)

    @property
    def orientation(self):
        return quat2rot(self.fbk.orientation)

    @property
    def zoom_level(self):
        return self.cmd.io.f.get_float(1)

    @zoom_level.setter
    def zoom_level(self, value):
        self.cmd.io.f.set_float(1, value)

    @property
    def spot_light(self):
        return self.cmd.io.f.get_float(2)

    @spot_light.setter
    def spot_light(self, value):
        self.cmd.io.f.set_float(2, value)

    @property
    def flood_light(self):
        return self.cmd.io.f.get_float(3)

    @flood_light.setter
    def flood_light(self, value):
        self.cmd.io.f.set_float(3, value)