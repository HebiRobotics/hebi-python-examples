from __future__ import annotations

import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import socket

import hebi
from hebi.arm import GroupManager

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.group import Group


class HebiCamera(GroupManager):
    def __init__(self, group: Group):
        super().__init__(time.time, group)

        self.roll: float = np.nan #self.ea_zxz[0]
        #self.set_on_update_callback(self.update_image_rotation)

    @staticmethod
    def from_family(family: str):
        lookup = hebi.Lookup()
        group = lookup.get_group_from_names(family, ['toolhead', 'cam1'], timeout_ms=2000)
        if group is None:
            raise RuntimeError(f'Could not find HEBI Camera with family {family}')
        return HebiCamera(group)

    def stream_to(self, target: str, bitrate_kbps: int | None = None):
        ip_cmd = hebi.GroupCommand(self.group.size)
        ip_cmd[1].ip_address = socket.gethostbyname(target)
        if bitrate_kbps is not None:
            ip_cmd[1].io.b.set_int(5, bitrate_kbps)
        self.group.send_command(ip_cmd)

    #def update_image_rotation(self):
    #    ea = self.ea_zxz
    #    roll_imu = ea[0]
    #    pitch_check = ea[1]

    #    roll_gyro = self.roll + self.last_feedback[0].gyro[2] * self.dt

    #    # Singularity Check - Video Rotation
    #    dead_zone_range = np.deg2rad(10)
    #    smooth_zone_range = np.deg2rad(40)
    #    alpha_range = smooth_zone_range - dead_zone_range

    #    if pitch_check < dead_zone_range or pitch_check > np.pi - dead_zone_range:
    #        self.roll = roll_gyro

    #    elif pitch_check <= smooth_zone_range or pitch_check >= np.pi - smooth_zone_range:
    #        if pitch_check > np.pi / 2:
    #            pitch_check = np.pi - pitch_check

    #        alpha = abs(pitch_check - dead_zone_range) / alpha_range

    #        self.roll = alpha * roll_imu + (1 - alpha) * roll_gyro

    #    else:
    #        self.roll = roll_imu

    #    return True

    #@property
    #def orientation(self):
    #    wxyz = self.last_feedback.orientation[0]
    #    xyzw = [*wxyz[1:], wxyz[0]]
    #    return R.from_quat(xyzw).as_matrix()

    #@property
    #def ea_zxz(self) -> npt.NDArray[np.float64]:
    #    wxyz = self.last_feedback.orientation[0]
    #    return R.from_quat(wxyz, scalar_first=True).as_euler('zxz')

    @property
    def light_enable(self):
        return self.pending_command[0].io.e.get_int(1)

    @light_enable.setter
    def light_enable(self, value: bool):
        self.pending_command[0].io.e.set_int(1, 1 if value else 0)

    @property
    def light_pwm(self):
        return self.pending_command[0].io.f.get_float(1)

    @light_pwm.setter
    def light_pwm(self, value: float):
        self.pending_command[0].io.f.set_float(1, value)

    @property
    def zoom_level(self):
        return self.pending_command[1].io.c.get_float(3)

    @zoom_level.setter
    def zoom_level(self, value):
        self.pending_command[1].io.c.set_float(3, value)

    def set_overlay_text(self, txt: str):
        overlay_cmd = hebi.GroupCommand(self.group.size)
        overlay_cmd.append_log = txt
        self.group.send_command(overlay_cmd)

    def clear_overlay_text(self):
        self.set_overlay_text('')
