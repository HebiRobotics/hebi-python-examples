"""Core sampler payload for Treadward: a pivoting mast, a two-motor chain drive, and a wiggler tool.

States: Startup -> Stowed <-> Deploying -> Deployed <-> Stowing -> Stowed.
Nothing moves on startup. After a disconnect or M-stop the sampler resumes
the state it was in (re-planning from current positions).

Coupling with the base (who may deploy/stow/drive when) is supplied through
``CoreSamplerInputs`` by the demo (see ``smart_treadward_sampler_control.py``).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import hebi
import numpy as np
from hebi._internal.ffi._marshalling import CommandIoBankField
from hebi.arm import GroupManager

from kits.common.state_machine import Controller, ControllerState

if TYPE_CHECKING:
    from hebi._internal.group import Group
    from numpy.typing import NDArray


def _near(value, target, tolerance) -> bool:
    return abs(float(np.mean(value)) - float(np.mean(target))) < tolerance


class CoreSampler(GroupManager):
    MAST_DEPLOY_POS = (np.pi / 2,)  # rad
    MAST_STOW_POS = (0.09,)  # rad
    MAX_MAST_ROT = (np.pi / 2) / 22  # rad/s
    MAST_TOLERANCE = 0.02  # rad from deployed/stowed target while still considered deployed/stowed

    CHAIN_MAX_POS = (31.05, 31.04)
    CHAIN_MIN_POS = (-0.05, -0.06)
    CHAIN_STOW_POS = (-0.05, -0.06)
    CHAIN_RAMP_TIME = 0.1  # s
    CHAIN_TOLERANCE = 0.05  # rad from stowed target while still considered stowed

    MAX_CHAIN_SPEED = 1 / 60  # m/s
    CHAIN_SPROCKET_RADIUS = 0.044  # m
    MAX_CHAIN_ROT = MAX_CHAIN_SPEED / CHAIN_SPROCKET_RADIUS  # rad/s

    CHAIN_VEL_SCALE_MARGIN = 0.3  # rad from the ends before reducing max speed

    MAX_STOW_WAIT_TIME = 2  # s to wait for stow to become safe before giving up
    MAX_DEPLOY_WAIT_TIME = 2  # s to wait for deploy to become safe before giving up

    WIGGLER_IO_BANK = CommandIoBankField.E
    WIGGLER_IO_PIN = 8

    def __init__(self, group: Group):
        super().__init__(time.time, group)

        self.mast_fbk = self.last_feedback.create_view([0])
        self.chain_fbk = self.last_feedback.create_view([1, 2])
        self.mast_cmd = self.pending_command.create_view([0])
        self.chain_cmd = self.pending_command.create_view([1, 2])
        self.tool_cmd = self.pending_command.create_view([3])

        self.mast_traj = None
        self.chain_traj = None
        self.wiggle = False

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    @property
    def has_active_mast_trajectory(self):
        return self.mast_traj is not None and self._last_time < self.mast_traj.end_time

    @property
    def has_active_chain_trajectory(self):
        return self.chain_traj is not None and self._last_time < self.chain_traj.end_time

    @property
    def has_active_trajectory(self):
        return self.has_active_mast_trajectory or self.has_active_chain_trajectory

    def _target(self, traj, fbk) -> NDArray[np.float64]:
        """Where a joint is being commanded to right now (trajectory), else where it is (feedback)."""
        if traj is not None:
            return traj.get_state(min(self._last_time, traj.end_time))[0]
        return np.asarray(fbk.position)

    @property
    def mast_position(self):
        return self._target(self.mast_traj, self.mast_fbk)

    @property
    def chain_position(self):
        return self._target(self.chain_traj, self.chain_fbk)

    @property
    def mast_stowed(self):
        return _near(self.mast_position, self.MAST_STOW_POS, self.MAST_TOLERANCE)

    @property
    def mast_deployed(self):
        return _near(self.mast_position, self.MAST_DEPLOY_POS, self.MAST_TOLERANCE)

    @property
    def chain_stowed(self):
        return _near(self.chain_position, self.CHAIN_STOW_POS, self.CHAIN_TOLERANCE)

    # ------------------------------------------------------------------
    # Control thread
    # ------------------------------------------------------------------

    def _inner_update(self, t: float) -> bool:
        res = super()._inner_update(t)

        for traj, cmd in ((self.mast_traj, self.mast_cmd), (self.chain_traj, self.chain_cmd)):
            if traj is None:
                cmd.velocity = 0.0
            else:
                pos, vel, _ = traj.get_state(min(t, traj.end_time))
                cmd.position = pos
                cmd.velocity = vel if t < traj.end_time else 0.0

        self.tool_cmd.io.set_int(self.WIGGLER_IO_BANK, self.WIGGLER_IO_PIN, int(self.wiggle))
        return res

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def hold(self, ramp_time: float = 0.25):
        """Ramp mast and chain to a stop where they are being commanded (or are)."""
        t_now = time.time()
        self.set_mast_trajectory(t_now, ramp_time, p=self.mast_position)
        self.set_chain_trajectory(t_now, ramp_time, p=self.chain_position)

    def clear_motion(self):
        """Drop mast/chain targets and stop the wiggler; the next update commands zero velocity."""
        self.mast_traj = None
        self.chain_traj = None
        self.wiggle = False

    def set_mast_trajectory(self, t_now: float, ramp_time: float, p=None, v=None):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((1, 2), dtype=np.float64)
        velocities = np.empty((1, 2), dtype=np.float64)
        accelerations = np.empty((1, 2), dtype=np.float64)

        if self.mast_traj is not None:
            t = min(t_now, self.mast_traj.end_time)
            positions[:, 0], velocities[:, 0], accelerations[:, 0] = self.mast_traj.get_state(t)
        else:
            positions[:, 0] = self.mast_fbk.position
            velocities[:, 0] = self.mast_fbk.velocity
            accelerations[:, 0] = 0.0

        positions[:, 1] = np.nan if p is None else p
        velocities[:, 1] = 0.0 if v is None else v
        accelerations[:, 1] = 0.0

        self.mast_traj = hebi.trajectory.create_trajectory(times, positions, velocities, accelerations)

    def set_chain_trajectory(self, t_now: float, ramp_time: float, p=None, v=None, trans_v=None):
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

        self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities, accelerations)

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
            distance_to_max = np.min(np.asarray(self.CHAIN_MAX_POS) - positions[:, 0])
            vel_scalar = float(np.min([distance_to_max / self.CHAIN_VEL_SCALE_MARGIN, 1]))
        elif np.mean(v) < 0:
            distance_to_min = np.min(positions[:, 0] - np.asarray(self.CHAIN_MIN_POS))
            vel_scalar = float(np.min([distance_to_min / self.CHAIN_VEL_SCALE_MARGIN, 1]))

        positions[:, 1] = np.nan
        velocities[:, 1] = v * vel_scalar

        self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities)
        target_positions, _, _ = self.chain_traj.get_state(self.chain_traj.end_time)

        # Clamp a trajectory that would leave the chain's range
        if np.any(target_positions < self.CHAIN_MIN_POS):
            positions[:, 1] = self.CHAIN_MIN_POS
            self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities)
        elif np.any(target_positions > self.CHAIN_MAX_POS):
            positions[:, 1] = self.CHAIN_MAX_POS
            self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities)

    def deploy_mast(self, t_now: float):
        print("Now deploying mast")
        deploy_time = abs(np.mean(self.MAST_DEPLOY_POS) - np.mean(self.mast_position)) / self.MAX_MAST_ROT + 2
        self.set_mast_trajectory(t_now, float(deploy_time), p=self.MAST_DEPLOY_POS)
        self.set_chain_vel_trajectory(t_now, self.CHAIN_RAMP_TIME, v=[0])

    def stow_mast(self, t_now: float):
        print("Now stowing mast")
        stow_time = abs(np.mean(self.MAST_STOW_POS) - np.mean(self.mast_position)) / self.MAX_MAST_ROT + 2
        self.set_mast_trajectory(t_now, float(stow_time), p=self.MAST_STOW_POS)
        self.set_chain_vel_trajectory(t_now, self.CHAIN_RAMP_TIME, v=[0])

    def stow_chain(self, t_now: float):
        print("Now stowing chain")
        stow_distance = float(np.mean(self.CHAIN_STOW_POS) - np.mean(self.chain_position))
        stow_vel = [self.MAX_CHAIN_ROT if stow_distance > 0 else -self.MAX_CHAIN_ROT] * 2
        stow_time = abs(stow_distance) / self.MAX_CHAIN_ROT + 2
        self.set_chain_trajectory(t_now, stow_time, p=self.CHAIN_STOW_POS, trans_v=stow_vel)

    # ------------------------------------------------------------------
    # Direct commands (outside the control loop)
    # ------------------------------------------------------------------

    def set_color(self, color: hebi.Color | str):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        self.set_color(hebi.Color(0, 0, 0, 0))

    def stop_wiggler(self):
        """Turn the wiggler off with a direct command. Use after the control thread has stopped."""
        self.wiggle = False
        cmd = hebi.GroupCommand(self.group.size)
        cmd[3].io.set_int(self.WIGGLER_IO_BANK, self.WIGGLER_IO_PIN, 0)
        self.group.send_command(cmd)


@dataclass
class CoreSamplerInputs:
    deploy: bool = False
    stow: bool = False
    chain: float = 0.0
    wiggle_mode: bool = False
    wiggle_toggle: bool = False
    override_startup: bool = False

    # Coupling with the base, filled in by the demo from the base's state.
    deploy_safe: bool = False
    stow_safe: bool = False
    allow_startup: bool = False


# ----------------------------------------------------------------------
# States
# ----------------------------------------------------------------------


class _ResumeSame:
    """Sampler states resume where they were after a disconnect or M-stop (re-planning from current positions)."""

    controller: CoreSamplerController

    def resume_state(self):
        return type(self)(self.controller)


@dataclass(eq=False)
class SamplerStartupState(_ResumeSame, ControllerState["CoreSamplerController"]):
    """Hold still until mast and chain are stowed (or overridden) and the base allows startup."""

    held: bool = field(default=False, init=False)
    allow_base_startup: bool = field(default=False, init=False)
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def update(self, t_now, **kwargs):
        if not self.held:
            # First tick with feedback: hold where we are. Never move on startup.
            self.controller.sampler.hold()
            self.held = True
        requested, self.requested = self.requested, None
        return requested

    def receive_inputs(self, t_now, inputs: CoreSamplerInputs, **kwargs):
        sampler = self.controller.sampler
        if (sampler.chain_stowed and sampler.mast_stowed) or inputs.override_startup:
            self.allow_base_startup = True
        if self.allow_base_startup and inputs.allow_startup:
            self.requested = SamplerStowedState(self.controller)


@dataclass(eq=False)
class SamplerStowedState(_ResumeSame, ControllerState["CoreSamplerController"]):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state, **kwargs):
        self.controller.sampler.clear_color()

    def receive_inputs(self, t_now, inputs: CoreSamplerInputs, **kwargs):
        if inputs.deploy:
            self.requested = SamplerDeployingState(self.controller)

    def update(self, t_now, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class SamplerDeployingState(_ResumeSame, ControllerState["CoreSamplerController"]):
    timeout: float = field(default=0.0, init=False)

    def on_enter(self, from_state, **kwargs):
        self.controller.sampler.set_color("magenta")
        self.timeout = time.time() + self.controller.sampler.MAX_DEPLOY_WAIT_TIME

    def update(self, t_now, **kwargs):
        c = self.controller
        if c.deploy_safe:
            if not c.sampler.has_active_trajectory:
                if not c.sampler.mast_deployed:
                    c.sampler.deploy_mast(t_now)
                else:
                    return SamplerDeployedState(c)
        elif t_now > self.timeout:
            print(f"{c.prefix}Abandoning deploy, unsafe")
            return SamplerStowedState(c)
        return None


@dataclass(eq=False)
class SamplerDeployedState(_ResumeSame, ControllerState["CoreSamplerController"]):
    # Set by receive_inputs; update() returns it (inputs never transition directly).
    requested: ControllerState | None = field(default=None, init=False)

    def on_enter(self, from_state, **kwargs):
        self.controller.sampler.clear_color()

    def receive_inputs(self, t_now, inputs: CoreSamplerInputs, **kwargs):
        sampler = self.controller.sampler
        if self.requested is not None:
            return
        if inputs.stow:
            self.requested = SamplerStowingState(self.controller)
            return

        if inputs.wiggle_toggle:
            sampler.wiggle = bool(inputs.wiggle_mode)
            print("Begin wiggle" if sampler.wiggle else "End wiggle")

        target_chain_vel = -inputs.chain * sampler.MAX_CHAIN_ROT
        sampler.set_chain_vel_trajectory(t_now, sampler.CHAIN_RAMP_TIME, [target_chain_vel] * 2)

    def update(self, t_now, **kwargs):
        requested, self.requested = self.requested, None
        return requested


@dataclass(eq=False)
class SamplerStowingState(_ResumeSame, ControllerState["CoreSamplerController"]):
    timeout: float = field(default=0.0, init=False)

    def on_enter(self, from_state, **kwargs):
        self.controller.sampler.set_color("magenta")
        self.controller.sampler.wiggle = False
        self.timeout = time.time() + self.controller.sampler.MAX_STOW_WAIT_TIME

    def update(self, t_now, **kwargs):
        c = self.controller
        if c.stow_safe:
            if not c.sampler.has_active_trajectory:
                if not c.sampler.chain_stowed:
                    c.sampler.stow_chain(t_now)
                elif not c.sampler.mast_stowed:
                    c.sampler.stow_mast(t_now)
                else:
                    return SamplerStowedState(c)
        elif t_now > self.timeout:
            print(f"{c.prefix}Abandoning stow, unsafe")
            return SamplerDeployedState(c)
        return None


@dataclass(eq=False)
class SamplerFaultState(ControllerState["CoreSamplerController"]):
    # Drop targets: modules ignore commands while M-stopped, and a held target
    # would be driven to on release. The resumed state re-plans from feedback.
    def on_enter(self, from_state, **kwargs):
        self.controller.sampler.clear_motion()


@dataclass(eq=False)
class SamplerExitState(ControllerState["CoreSamplerController"]):
    def on_enter(self, from_state, **kwargs):
        self.controller.sampler.wiggle = False
        self.controller.sampler.hold()


class CoreSamplerController(Controller[CoreSampler, CoreSamplerInputs]):
    """Core sampler payload. After a disconnect or M-stop it resumes its previous state."""

    def __init__(self, sampler: CoreSampler, *, name: str = "Payload", input_timeout: float | None = 1.0):
        super().__init__(sampler, SamplerStartupState, SamplerFaultState, SamplerExitState, name=name, input_timeout=input_timeout)
        # Latest coupling flags from the base, read by the deploy/stow states each tick.
        self.deploy_safe = False
        self.stow_safe = False

    @property
    def sampler(self) -> CoreSampler:
        return self.manager

    @property
    def base_drive_safe(self) -> bool:
        """Whether the base may drive: payload stowed, or deployed with the chain retracted."""
        match self.state:
            case SamplerStowedState():
                return True
            case SamplerDeployedState():
                return self.sampler.chain_stowed
        return False

    @property
    def allow_base_startup(self) -> bool:
        state = self.state
        return not isinstance(state, SamplerStartupState) or state.allow_base_startup

    def receive_inputs(self, inputs: CoreSamplerInputs) -> None:
        with self.manager.lock:
            self.deploy_safe = inputs.deploy_safe
            self.stow_safe = inputs.stow_safe
            super().receive_inputs(inputs)

    def on_disconnect(self) -> None:
        self.sampler.wiggle = False
        self.sampler.hold()

    def stop(self) -> None:
        super().stop()
        # The control thread is stopped now; make sure the wiggler is off.
        self.sampler.stop_wiggler()
