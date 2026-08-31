import sys
from time import time, sleep
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

from hebi._internal.ffi._marshalling import CommandIoBankField, FeedbackIoBankField

import typing


if typing.TYPE_CHECKING:
    from typing import Optional, Callable
    import numpy.typing as npt
    from hebi._internal.group import Group
    from hebi._internal.mobile_io import MobileIO


class CoreSampler:
    MAST_DEPLOY_POS = [np.pi / 2]  # rad
    MAST_STOW_POS = [0.09]  # rad
    MAX_MAST_ROT = (np.pi / 2) / 22  # rad/s
    MAST_TOLERANCE = 0.02  # The distance (in rad) we may be away from deployed/stowed target while still considering ourselves deployed/stowed

    CHAIN_MAX_POS = [31.05, 31.04]
    CHAIN_MIN_POS = [-0.05, -0.06]
    CHAIN_STOW_POS = [-0.05, -0.06]
    CHAIN_RAMP_TIME = 0.1  # s
    CHAIN_TOLERANCE = 0.05  # The distance (in rad) we may be away from stowed target while still considering ourselves stowed

    CHAIN_VEL_SCALE = 1
    MAX_CHAIN_SPEED = 1 / 60  # m/s
    CHAIN_SPROCKET_RADIUS = 0.044  # m
    MAX_CHAIN_ROT = MAX_CHAIN_SPEED / CHAIN_SPROCKET_RADIUS  # rad/s

    CHAIN_VEL_SCALE_MARGIN = (
        0.3  # The distance (in rad) away from the edges before reducing max speed
    )

    MAX_STOW_WAIT_TIME = 2  # How long will we wait before giving up on stowing
    MAX_DEPLOY_WAIT_TIME = 2  # How long will we wait before giving up on deploying

    WIGGLER_IO_BANK = CommandIoBankField.E
    WIGGLER_IO_PIN = 8

    def __init__(self, group: "Group"):
        self.group = group

        fbk = self.group.get_next_feedback()
        while fbk is None:
            fbk = self.group.get_next_feedback()
        self.fbk = fbk

        self.mast_fbk = self.fbk.create_view([0])
        self.chain_fbk = self.fbk.create_view([1, 2])
        self.tool_fbk = self.fbk.create_view([3])
        self.cmd = hebi.GroupCommand(group.size)
        self.mast_cmd = self.cmd.create_view([0])
        self.chain_cmd = self.cmd.create_view([1, 2])
        self.tool_cmd = self.cmd.create_view([3])

        self.cmd.position = self.fbk.position

        self.t_prev: float = time()

        self.mast_traj = None
        self.chain_traj = None
        self.wiggle = False

        self.robot_model = None

        self.deploy_safe = True
        self.cancel_deploy = False
        self.stow_safe = True
        self.cancel_stow = False

        self.base_deploy_safe = True
        self.base_stow_safe = False

        self.ignore_estop = False  # ALWAYS SET FALSE BEFORE RUNNING ON REAL HARDWARE (True for imitation group testing only)

    @property
    def mstop_pressed(self):
        return any(self.fbk.mstop_state == 0)

    @property
    def has_active_mast_trajectory(self):
        if self.mast_traj is not None and self.t_prev < self.mast_traj.end_time:
            return True
        return False

    @property
    def has_active_chain_trajectory(self):
        if self.chain_traj is not None and self.t_prev < self.chain_traj.end_time:
            return True
        return False

    @property
    def has_active_trajectory(self):
        return self.has_active_mast_trajectory or self.has_active_chain_trajectory

    @property
    def mast_stowed(self):
        if self.mast_cmd is not None:
            return np.mean(self.MAST_STOW_POS) - self.MAST_TOLERANCE < np.mean(
                self.mast_cmd.position
            ) and np.mean(self.MAST_STOW_POS) + self.MAST_TOLERANCE > np.mean(
                self.mast_cmd.position
            )
        return np.mean(self.MAST_STOW_POS) - self.MAST_TOLERANCE < np.mean(
            self.mast_fbk.position
        ) and np.mean(self.MAST_STOW_POS) + self.MAST_TOLERANCE > np.mean(
            self.mast_fbk.position
        )

    @property
    def mast_deployed(self):
        if self.mast_cmd is not None:
            return np.mean(self.MAST_DEPLOY_POS) - self.MAST_TOLERANCE < np.mean(
                self.mast_cmd.position
            ) and np.mean(self.MAST_DEPLOY_POS) + self.MAST_TOLERANCE > np.mean(
                self.mast_cmd.position
            )
        return np.mean(self.MAST_DEPLOY_POS) - self.MAST_TOLERANCE < np.mean(
            self.mast_fbk.position
        ) and np.mean(self.MAST_DEPLOY_POS) + self.MAST_TOLERANCE > np.mean(
            self.mast_fbk.position
        )

    @property
    def chain_stowed(self):
        if self.chain_cmd is not None:
            return np.mean(self.CHAIN_STOW_POS) - self.CHAIN_TOLERANCE < np.mean(
                self.chain_cmd.position
            ) and np.mean(self.CHAIN_STOW_POS) + self.CHAIN_TOLERANCE > np.mean(
                self.chain_cmd.position
            )
        return np.mean(self.CHAIN_STOW_POS) - self.CHAIN_TOLERANCE < np.mean(
            self.chain_fbk.position
        ) and np.mean(self.CHAIN_STOW_POS) + self.CHAIN_TOLERANCE > np.mean(
            self.chain_fbk.position
        )

    def update_feedback(self):
        self.group.get_next_feedback(reuse_fbk=self.fbk)

    def update(self, t_now: float, get_feedback: bool = True):
        if get_feedback:
            self.group.get_next_feedback(reuse_fbk=self.fbk)

        if not self.has_active_trajectory:
            self.cmd.velocity = 0.0
        else:
            if self.mast_traj is not None:
                t = min(t_now, self.mast_traj.end_time)
                [pos, vel, _] = self.mast_traj.get_state(t)

                self.mast_cmd.velocity = vel
                self.mast_cmd.position = pos

            if self.chain_traj is not None:
                t = min(t_now, self.chain_traj.end_time)
                [pos, vel, _] = self.chain_traj.get_state(t)
                self.chain_cmd.velocity = vel
                self.chain_cmd.position = pos

        self.tool_cmd.io.set_int(
            self.WIGGLER_IO_BANK, self.WIGGLER_IO_PIN, int(self.wiggle)
        )
        self.t_prev = t_now

    def send(self):
        self.group.send_command(self.cmd)

    def set_mast_cmd(self, p=None, v=None, e=None):
        if p is not None:
            self.mast_cmd.position = p
        if v is not None:
            self.mast_cmd.velocity = v
        if e is not None:
            self.mast_cmd.effort = e

    def set_chain_cmd(self, p=None, v=None, e=None):
        if p is not None:
            self.chain_cmd.position = p
        if v is not None:
            self.chain_cmd.velocity = v
        if e is not None:
            self.chain_cmd.effort = e

    def set_mast_trajectory(self, t_now: float, ramp_time: float, p=None, v=None):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((1, 2), dtype=np.float64)
        velocities = np.empty((1, 2), dtype=np.float64)
        accelerations = np.empty((1, 2), dtype=np.float64)

        if self.mast_traj is not None:
            t = min(t_now, self.mast_traj.end_time)
            positions[:, 0], velocities[:, 0], accelerations[:, 0] = (
                self.mast_traj.get_state(t)
            )
        else:
            positions[:, 0] = self.mast_fbk.position
            velocities[:, 0] = self.mast_fbk.velocity
            accelerations[:, 0] = self.mast_fbk.effort_command

        positions[:, 1] = np.nan if p is None else p
        velocities[:, 1] = 0.0 if v is None else v
        accelerations[:, 1] = 0.0

        self.mast_traj = hebi.trajectory.create_trajectory(
            times, positions, velocities, accelerations
        )

    def set_chain_trajectory(
        self, t_now: float, ramp_time: float, p=None, v=None, trans_v=None
    ):
        if ramp_time < 3:
            times = [t_now, t_now + ramp_time]
            positions = np.full((2, 2), np.nan)
            velocities = np.full((2, 2), np.nan)
            accelerations = np.full((2, 2), np.nan)
        else:
            times = [t_now, t_now + ramp_time / 2, t_now + ramp_time]
            positions = np.full((2, 3), np.nan)
            velocities = np.full((2, 3), np.nan)
            accelerations = np.full((2, 3), np.nan)

        if self.chain_traj is not None:
            t = min(t_now, self.chain_traj.end_time)
            positions[:, 0], velocities[:, 0], _ = self.chain_traj.get_state(t)
        else:
            positions[:, 0] = self.chain_fbk.position
            velocities[:, 0] = self.chain_fbk.velocity

        if ramp_time < 3:
            positions[:, 1] = np.nan if p is None else p
            velocities[:, 1] = 0.0 if v is None else v
        else:
            positions[:, 2] = np.nan if p is None else p

            velocities[:, 2] = 0.0 if v is None else v
            velocities[:, 1] = np.nan if trans_v is None else trans_v

            accelerations[:, 1] = 0.0

        self.chain_traj = hebi.trajectory.create_trajectory(
            times, positions, velocities, accelerations
        )

    def set_chain_vel_trajectory(self, t_now: float, ramp_time: float, v=None):
        times = [t_now, t_now + ramp_time]
        positions = np.full((2, 2), np.nan)
        velocities = np.full((2, 2), np.nan)
        v = np.array(v)
        vel_scalar = 0.0

        if self.chain_traj is not None:
            t = min(t_now, self.chain_traj.end_time)
            positions[:, 0], velocities[:, 0], _ = self.chain_traj.get_state(t)
        else:
            positions[:, 0] = self.chain_fbk.position
            velocities[:, 0] = self.chain_fbk.velocity

        # Reduce velocity when driving towards edge of workspace
        if np.mean(v) > 0:
            distance_to_max = np.min(self.CHAIN_MAX_POS - positions[:, 0])
            vel_scalar = float(
                np.min([distance_to_max / self.CHAIN_VEL_SCALE_MARGIN, 1])
            )
        elif np.mean(v) < 0:
            distance_to_min = np.min(positions[:, 0] - self.CHAIN_MIN_POS)
            vel_scalar = float(
                np.min([distance_to_min / self.CHAIN_VEL_SCALE_MARGIN, 1])
            )

        positions[:, 1] = np.nan
        velocities[:, 1] = v * vel_scalar

        self.chain_traj = hebi.trajectory.create_trajectory(
            times, positions, velocities
        )
        target_positions, _, _ = self.chain_traj.get_state(self.chain_traj.end_time)

        # Check trajectory outside of bounds
        below_min = False
        above_max = False

        for i in range(2):
            if target_positions[i] < self.CHAIN_MIN_POS[i]:
                below_min = True
            if target_positions[i] > self.CHAIN_MAX_POS[i]:
                above_max = True

        if below_min:
            positions[:, 1] = self.CHAIN_MIN_POS
            self.chain_traj = hebi.trajectory.create_trajectory(
                times, positions, velocities
            )
        elif above_max:
            positions[:, 1] = self.CHAIN_MAX_POS
            self.chain_traj = hebi.trajectory.create_trajectory(
                times, positions, velocities
            )

    def deploy_mast(self, t_now: float):
        if self.deploy_safe:
            print("Now deploying mast")

            if self.mast_cmd is not None:
                mast_pos = self.mast_cmd.position
            else:
                mast_pos = self.mast_fbk.position

            deploy_distance = float(
                abs(np.mean(self.MAST_DEPLOY_POS) - np.mean(mast_pos))
            )
            deploy_time = deploy_distance / self.MAX_MAST_ROT + 2

            self.set_mast_trajectory(t_now, deploy_time, p=self.MAST_DEPLOY_POS)
            self.set_chain_vel_trajectory(t_now, self.CHAIN_RAMP_TIME, v=[0])
        else:
            print("Unsafe to deploy mast")

    def stow_mast(self, t_now: float):
        if self.stow_safe:
            print("Now stowing mast")

            if self.mast_cmd is not None:
                mast_pos = self.mast_cmd.position
            else:
                mast_pos = self.mast_fbk.position

            stow_distance = float(abs(np.mean(self.MAST_STOW_POS) - np.mean(mast_pos)))
            stow_time = stow_distance / self.MAX_MAST_ROT + 2

            self.set_mast_trajectory(t_now, stow_time, p=self.MAST_STOW_POS)
            self.set_chain_vel_trajectory(t_now, self.CHAIN_RAMP_TIME, v=[0])
        else:
            print("Unsafe to stow mast")

    def stow_chain(self, t_now: float):
        print("Now stowing chain")

        if self.chain_cmd is not None:
            chain_pos = self.chain_cmd.position
        else:
            chain_pos = self.chain_fbk.position

        stow_distance = float(np.mean(self.CHAIN_STOW_POS) - np.mean(chain_pos))

        if stow_distance > 0:
            stow_vel = [self.MAX_CHAIN_ROT] * 2
        else:
            stow_vel = [-self.MAX_CHAIN_ROT] * 2

        stow_distance = abs(stow_distance)
        stow_time = stow_distance / self.MAX_CHAIN_ROT + 2

        self.set_chain_trajectory(
            t_now, stow_time, p=self.CHAIN_STOW_POS, trans_v=stow_vel
        )

    def set_robot_model(self, hrdf_file: str):
        self.robot_model = hebi.robot_model.import_from_hrdf(hrdf_file)

    def set_color(self, color: "hebi.Color | str"):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = hebi.Color(0, 0, 0, 0)
        self.group.send_command(color_cmd)

    def stop_wiggler(self):
        self.tool_cmd.io.set_int(self.WIGGLER_IO_BANK, self.WIGGLER_IO_PIN, 0)
        self.send()


class CoreSamplerControlState(Enum):
    INITIAL = auto()
    STARTUP = auto()
    DEPLOYING = auto()
    STOWING = auto()
    DEPLOYED = auto()
    STOWED = auto()
    DISCONNECTED = auto()
    EMERGENCY_STOP = auto()
    EXIT = auto()

    @property
    def is_error_state(self):
        return self in [
            CoreSamplerControlState.DISCONNECTED,
            CoreSamplerControlState.EMERGENCY_STOP,
        ]


class CoreSamplerInputs:
    def __init__(
        self,
        deploy: bool = False,
        deploy_safe: bool = False,
        stow: bool = False,
        stow_safe: bool = False,
        pivot_adjust: float = 0.0,
        chain: float = 0.0,
        wiggle_mode: bool = False,
        wiggle_toggle: bool = False,
        base_deployed: bool = False,
        override_startup: bool = False,
        allow_startup: bool = False,
    ):
        self.deploy = deploy
        self.deploy_safe = deploy_safe
        self.stow = stow
        self.stow_safe = stow_safe
        self.pivot_adjust = pivot_adjust
        self.chain = chain
        self.wiggle_mode = wiggle_mode
        self.wiggle_toggle = wiggle_toggle
        self.base_deployed = base_deployed
        self.override_startup = override_startup
        self.allow_startup = allow_startup

    def __repr__(self) -> str:
        return (
            f"CoreSamplerInputs(deploy={self.deploy}, deploy_safe={self.deploy_safe}, stow={self.stow}, stow_safe={self.stow_safe}, "
            f"pivot_adjust={self.pivot_adjust}, chain={self.chain}, "
            f"wiggle_mode={self.wiggle_mode}, wiggle_toggle={self.wiggle_toggle}, base_deployed={self.base_deployed})"
            f"override_startup={self.override_startup}, allow_startup={self.allow_startup}"
        )


class CoreSamplerControl:
    def __init__(self, sampler: CoreSampler):
        self.namespace = ""

        self.state = CoreSamplerControlState.INITIAL
        self.prev_state = self.state
        self.sampler = sampler

        self.stow_timeout_time = 0.0
        self.deploy_timeout_time = 0.0

        self._transition_handlers: "list[Callable[[CoreSamplerControl, CoreSamplerControlState], None]]" = []
        self._update_handlers: "list[Callable[[CoreSamplerControl, CoreSamplerControlState], None]]" = []

        self.last_cmd_t = time()

        self.allow_base_startup = False
        self.base_drive_safe = False

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def start_logging(self):
        self.sampler.group.start_log("logs", mkdirs=True)

    def cycle_log(self):
        self.sampler.group.stop_log()
        self.sampler.group.start_log("logs", mkdirs=True)

    def send(self):
        self.sampler.send()

    def update(
        self,
        t_now: float,
        m_update: bool,
        sampler_input: "Optional[CoreSamplerInputs]" = None,
    ):
        self.sampler.update_feedback()

        if self.state is self.state.EXIT:
            return

        if (
            self.sampler.mstop_pressed
            and self.state is not self.state.EMERGENCY_STOP
            and not self.sampler.ignore_estop
        ):
            self.transition_to(t_now, self.state.EMERGENCY_STOP)

        # This should never run. Just here to make the type checker happy.
        if sampler_input is None:
            print(self.namespace + "sampler input is None")
            return

        # Transition to disconnected if no mobile update recieved
        if not m_update:
            if not self.state.is_error_state and (t_now - self.last_cmd_t) > 1.0:
                print(self.namespace + "mobileIO timeout, payload disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
        else:
            # Reset the timeout
            self.last_cmd_t = t_now

        # Update deploy/stow safety
        self.sampler.deploy_safe = sampler_input.deploy_safe
        self.sampler.stow_safe = sampler_input.stow_safe

        if self.state is self.state.EMERGENCY_STOP:
            if not self.sampler.mstop_pressed:
                print(self.namespace + "Emergency Stop Released")
                self.transition_to(t_now, self.prev_state)

        # Transition to previous state if mobileIO is reconnected
        elif self.state is self.state.DISCONNECTED and m_update:
            self.last_cmd_t = t_now
            print(self.namespace + "Controller reconnected, demo continued.")
            self.transition_to(t_now, self.prev_state)

        # On first loop move to startup
        elif self.state is self.state.INITIAL:
            self.transition_to(t_now, self.state.STARTUP)

        # While stowing
        elif self.state is self.state.STOWING:
            if self.sampler.cancel_stow:
                self.sampler.cancel_stow = False
                self.transition_to(t_now, self.state.DEPLOYED)
            # If its safe to stow, begin stow sequence
            if self.sampler.stow_safe:
                if not self.sampler.has_active_trajectory:
                    if not self.sampler.chain_stowed:
                        self.sampler.stow_chain(t_now)
                    elif not self.sampler.mast_stowed:
                        self.sampler.stow_mast(t_now)
                    else:
                        self.transition_to(t_now, self.state.STOWED)
            # If it has been unsafe to stow for too long, return to deployed mode
            elif t_now > self.stow_timeout_time:
                self.transition_to(t_now, self.state.DEPLOYED)
                print(self.namespace + "Abandoning stow, unsafe")

        # While deploying
        elif self.state is self.state.DEPLOYING:
            if self.sampler.cancel_deploy:
                self.sampler.cancel_deploy = False
                self.transition_to(t_now, self.state.STOWED)
            # If it is safe to deploy, begin deploying
            if self.sampler.deploy_safe:
                if not self.sampler.has_active_trajectory:
                    if not self.sampler.mast_deployed:
                        self.sampler.deploy_mast(t_now)
                    else:
                        self.transition_to(t_now, self.state.DEPLOYED)
            # If it has been unsafe to deploy for too long, return to stowed mode
            elif t_now > self.deploy_timeout_time:
                self.transition_to(t_now, self.state.STOWED)
                print(self.namespace + "Abandoning deploy, unsafe")

        # Transition to stowed when startup complete
        elif self.state is self.state.STARTUP:
            if (
                (self.sampler.chain_stowed and self.sampler.mast_stowed)
                or sampler_input.override_startup
                or self.allow_base_startup
            ):
                self.allow_base_startup = True
                if sampler_input.allow_startup:
                    self.transition_to(t_now, self.state.STOWED)

        # Stowed mode
        elif self.state is self.state.STOWED:
            # Check for deploy button
            if sampler_input.deploy:
                self.transition_to(t_now, self.state.DEPLOYING)

        # Deployed mode
        elif self.state is self.state.DEPLOYED:
            # Check for stow button
            if sampler_input.stow:
                self.transition_to(t_now, self.state.STOWING)
            else:
                if sampler_input.wiggle_mode and sampler_input.wiggle_toggle:
                    print("Begin wiggle")
                    self.sampler.wiggle = True
                elif not sampler_input.wiggle_mode and sampler_input.wiggle_toggle:
                    print("End wiggle")
                    self.sampler.wiggle = False

                target_chain_vel = -sampler_input.chain * self.sampler.MAX_CHAIN_ROT
                self.sampler.set_chain_vel_trajectory(
                    t_now, self.sampler.CHAIN_RAMP_TIME, [target_chain_vel] * 2
                )

                self.base_drive_safe = self.sampler.chain_stowed

        self.sampler.update(t_now, get_feedback=False)

        for handler in self._update_handlers:
            handler(self, self.state)

    def transition_to(self, t_now: float, state: CoreSamplerControlState):
        if state == self.state:
            return
        self.prev_state = self.state

        if state is self.state.STARTUP:
            print(self.namespace + "PAYLOAD TRANSITIONING TO STARTUP")

        if state is self.state.DEPLOYING:
            print(self.namespace + "PAYLOAD TRANSITIONING TO DEPLOYING")
            self.deploy_timeout_time = t_now + self.sampler.MAX_DEPLOY_WAIT_TIME
            self.sampler.base_stow_safe = False
            self.base_drive_safe = False

        if state is self.state.DEPLOYED:
            print(self.namespace + "PAYLOAD TRANSITIONING TO DEPLOYED")
            self.base_drive_safe = False

        if state is self.state.STOWING:
            print(self.namespace + "PAYLOAD TRANSITIONING TO STOWING")
            self.sampler.wiggle = False
            self.stow_timeout_time = t_now + self.sampler.MAX_STOW_WAIT_TIME
            self.base_drive_safe = False

        if state is self.state.STOWED:
            print(self.namespace + "PAYLOAD TRANSITIONING TO STOWED")
            self.sampler.base_stow_safe = True
            self.base_drive_safe = True

        if state is self.state.DISCONNECTED:
            print(self.namespace + "PAYLOAD DISCONNECTED")
            self.sampler.mast_traj = None
            self.sampler.chain_traj = None
            self.sampler.wiggle = False
            self.base_drive_safe = False

        if state is self.state.EMERGENCY_STOP:
            print(self.namespace + "PAYLOAD EMERGENCY_STOP")
            self.sampler.mast_traj = None
            self.sampler.chain_traj = None
            self.sampler.wiggle = False
            self.base_drive_safe = False

        if state is self.state.EXIT:
            print(self.namespace + "PAYLOAD EXIT")
            self.sampler.stop_wiggler()

        for handler in self._transition_handlers:
            handler(self, state)

        self.state = state

    def stop(self):
        self.transition_to(time(), self.state.EXIT)
