
import hebi
from scipy.spatial.transform import Rotation as R
import numpy as np

import typing
if typing.TYPE_CHECKING:
    import numpy.typing as npt
    from hebi._internal.group import Group


class HebiCamera:
    def __init__(self, group: 'Group'):
        self.group = group
        self.fbk = self.group.get_next_feedback()
        while self.fbk is None:
            self.fbk = group.get_next_feedback()

        self.roll: float = self.ea_zxz[0]
        self.cmd = hebi.GroupCommand(self.group.size)

    def update(self):
        self.group.get_next_feedback(reuse_fbk=self.fbk)
        self.update_image_rotation()

    def send(self):
        self.group.send_command(self.cmd)

    def update_image_rotation(self):
        dt = 1.0 /self.group.feedback_frequency

        ea = self.ea_zxz
        roll_imu = ea[0]
        pitch_check = ea[1]

        roll_gyro = self.roll + self.fbk[0].gyro[2] * dt

        # Singularity Check - Video Rotation
        dead_zone_range = np.deg2rad(10)
        smooth_zone_range = np.deg2rad(40)
        alpha_range = smooth_zone_range - dead_zone_range

        if pitch_check < dead_zone_range or pitch_check > np.pi-dead_zone_range:
            self.roll = roll_gyro

        elif pitch_check <= smooth_zone_range or pitch_check >= np.pi-smooth_zone_range:
            if pitch_check > np.pi/2:
               pitch_check = np.pi - pitch_check

            alpha = abs(pitch_check - dead_zone_range) / alpha_range

            self.roll = alpha*roll_imu + (1-alpha)*roll_gyro

        else:
            self.roll = roll_imu

    @property
    def orientation(self):
        wxyz = self.fbk.orientation[0]
        xyzw = [*wxyz[1:], wxyz[0]]
        return R.from_quat(xyzw).as_matrix()

    @property
    def ea_zxz(self) -> 'npt.NDArray[np.float64]':
        wxyz = self.fbk.orientation[0]
        xyzw = [*wxyz[1:], wxyz[0]]
        return R.from_quat(xyzw).as_euler('zxz')

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