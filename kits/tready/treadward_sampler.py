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
    MAST_DEPLOY_POS = [np.pi/2] # rad
    MAST_STOW_POS = [0.07] # rad
    MAST_DEPLOY_TIME = 10.0 # s
    PIVOT_ADJUSTMENT_RANGE = 5 # degrees
    PIVOT_SCALE = np.radians(PIVOT_ADJUSTMENT_RANGE)
    PIVOT_VEL_SCALE = .1
    PIVOT_RAMP_TIME = 1 # s

    CHAIN_MAX_POS = [33.2490, 16.8126]
    CHAIN_MIN_POS = [-1.76, -18.18]
    CHAIN_STOW_POS = [0, 0] # add stow position for chain once known
    CHAIN_STOW_TIME = 10.0 # s
    CHAIN_RAMP_TIME = .1 # s

    CHAIN_VEL_SCALE = 1
    MAX_CHAIN_SPEED = 1/60 # m/s
    CHAIN_SPROCKET_RADIUS = .044 # m
    MAX_CHAIN_ROT = (MAX_CHAIN_SPEED / CHAIN_SPROCKET_RADIUS)

    CHAIN_VEL_SCALE_MARGIN = 2 # The distance (in rad) away from the edges before reducing max speed

    MAX_STOW_WAIT_TIME = 5 # How long will we wait before giving up on stowing
    MAX_DEPLOY_WAIT_TIME = 5 # How long will we wait before giving up on deploying

    WIGGLER_IO_BANK = CommandIoBankField.E
    WIGGLER_IO_PIN = 8


    def __init__(self, group: 'Group'):
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
        self.chain_stowed = True
        self.mast_stowed = True

        self.base_deploy_safe = True
        self.base_stow_safe = False

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
                [_, vel, _] = self.mast_traj.get_state(t)

                self.mast_cmd.velocity = vel
                self.mast_cmd.position += vel * (t_now - self.t_prev)
            
            if self.chain_traj is not None:
                t = min(t_now, self.chain_traj.end_time)
                [_, vel, _] = self.chain_traj.get_state(t)

                self.chain_cmd.velocity = vel
                self.chain_cmd.position += vel * (t_now - self.t_prev)

        self.tool_cmd.io.set_int(self.WIGGLER_IO_BANK, self.WIGGLER_IO_PIN, int(self.wiggle))
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
            positions[:, 0], velocities[:, 0], accelerations[:, 0] = self.mast_traj.get_state(t)
        else:
            positions[:, 0] = self.mast_fbk.position
            velocities[:, 0] = self.mast_fbk.velocity
            accelerations[:, 0] = self.mast_fbk.effort_command

        positions[:, 1] = np.nan if p is None else p
        velocities[:, 1] = 0.0 if v is None else v
        accelerations[:, 1] = 0.0

        self.mast_traj = hebi.trajectory.create_trajectory(times, positions, velocities, accelerations)

    def set_chain_trajectory(self, t_now: float, ramp_time: float, p=None, v=None):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((2, 2), dtype=np.float64)
        velocities = np.empty((2, 2), dtype=np.float64)

        if self.chain_traj is not None:
            t = min(t_now, self.chain_traj.end_time)
            positions[:, 0], velocities[:, 0], _ = self.chain_traj.get_state(t)
        else:
            positions[:, 0] = self.chain_fbk.position
            velocities[:, 0] = self.chain_fbk.velocity

        positions[:, 1] = np.nan if p is None else p
        velocities[:, 1] = 0.0 if v is None else v

        self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities)

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
            vel_scalar = float(np.min([distance_to_max/self.CHAIN_VEL_SCALE_MARGIN, 1]))
        elif np.mean(v) < 0:
            distance_to_min = np.min(positions[:, 0] - self.CHAIN_MIN_POS)
            vel_scalar = float(np.min([distance_to_min/self.CHAIN_VEL_SCALE_MARGIN, 1]))

        positions[:, 1] = np.nan
        velocities[:, 1] = v*vel_scalar

        self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities)
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
            self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities)
        elif above_max:
            positions[:, 1] = self.CHAIN_MAX_POS
            self.chain_traj = hebi.trajectory.create_trajectory(times, positions, velocities)
        
    def deploy_mast(self, t_now: float):
        if self.deploy_safe:
            print("Now deploying mast")
            self.set_mast_trajectory(t_now, self.MAST_DEPLOY_TIME, p=self.MAST_DEPLOY_POS)
            self.set_chain_vel_trajectory(t_now, self.CHAIN_RAMP_TIME, v=[0])
        else:
            print("Unsafe to deploy mast")
    
    def stow_mast(self, t_now: float):
        if self.stow_safe:
            print("Now stowing mast")
            self.set_mast_trajectory(t_now, self.MAST_DEPLOY_TIME, p=self.MAST_STOW_POS)
            self.set_chain_vel_trajectory(t_now, self.CHAIN_RAMP_TIME, v=[0])
        else:
            print("Unsafe to stow mast")
    
    def stow_chain(self, t_now: float):
        print("Now stowing chain")
        self.set_chain_trajectory(t_now, self.CHAIN_STOW_TIME, p=self.CHAIN_STOW_POS)

    def set_robot_model(self, hrdf_file: str):
        self.robot_model = hebi.robot_model.import_from_hrdf(hrdf_file)

    def set_color(self, color: 'hebi.Color | str'):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = hebi.Color(0, 0, 0, 0)
        self.group.send_command(color_cmd)


class CoreSamplerControlState(Enum):
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
        return self in [CoreSamplerControlState.DISCONNECTED, CoreSamplerControlState.EMERGENCY_STOP]


class CoreSamplerInputs:
    def __init__(self, deploy: bool = False, deploy_safe: bool = False, stow: bool = False, stow_safe: bool = False, 
                 pivot_adjust: float = 0.0, chain: float = 0.0, wiggle_mode: bool = False, wiggle_toggle: bool = False,
                 base_deployed: bool = False):
        self.deploy = deploy
        self.deploy_safe = deploy_safe
        self.stow = stow
        self.stow_safe = stow_safe
        self.pivot_adjust = pivot_adjust
        self.chain = chain
        self.wiggle_mode = wiggle_mode
        self.wiggle_toggle = wiggle_toggle
        self.base_deployed = base_deployed

    def __repr__(self) -> str:
        return (f'CoreSamplerInputs(deploy={self.deploy}, deploy_safe={self.deploy_safe}, stow={self.stow}, stow_safe={self.stow_safe}, 'f'pivot_adjust={self.pivot_adjust}, chain={self.chain}, 'f'wiggle_mode={self.wiggle_mode}, wiggle_toggle={self.wiggle_toggle}, base_deployed={self.base_deployed})')


class CoreSamplerControl:
    def __init__(self, sampler: CoreSampler):
        self.namespace = ''
        
        self.state = CoreSamplerControlState.STARTUP
        self.prev_state = self.state
        self.sampler = sampler
        
        self.stow_timeout_time = 0.0 
        self.deploy_timeout_time = 0.0

        self._transition_handlers: 'list[Callable[[CoreSamplerControl, CoreSamplerControlState], None]]' = []
        self._update_handlers: 'list[Callable[[CoreSamplerControl], None]]' = []

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

    def update(self, t_now: float, sampler_input: 'Optional[CoreSamplerInputs]'=None):
        self.sampler.update_feedback()

        if self.state is self.state.EXIT:
            return
        """
        if self.sampler.mstop_pressed and self.state is not self.state.EMERGENCY_STOP:
            self.prev_state = self.state
            self.transition_to(t_now, self.state.EMERGENCY_STOP)
            return
        """
        if sampler_input is None:
            if not self.state.is_error_state and (t_now - self.last_cmd_t) > 1.0:
                print(self.namespace + "mobileIO timeout, disabling motion")
                self.prev_state = self.state
                self.transition_to(t_now, self.state.DISCONNECTED)
            return
            
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
        elif self.state is self.state.DISCONNECTED:
            self.last_cmd_t = t_now
            print(self.namespace + 'Controller reconnected, demo continued.')
            self.transition_to(t_now, self.prev_state)
        
        # After startup, transition to stowing
        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.STOWING)

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
                        self.sampler.chain_stowed = True
                    elif not self.sampler.mast_stowed:
                        self.sampler.stow_mast(t_now)
                        self.sampler.mast_stowed = True
                    else:
                        self.transition_to(t_now, self.state.STOWED)
            # If it has been unsafe to stow for to long, return to deployed mode
            elif t_now > self.stow_timeout_time:
                self.transition_to(t_now, self.state.DEPLOYED)

        # While deploying
        elif self.state is self.state.DEPLOYING:
            if self.sampler.cancel_deploy:
                self.sampler.cancel_deploy = False
                self.transition_to(t_now, self.state.STOWED)
            # If it is safe to deploy, begin deploying
            if self.sampler.deploy_safe:
                if not self.sampler.has_active_trajectory:
                    if self.sampler.mast_stowed:
                        self.sampler.deploy_mast(t_now)
                        self.sampler.mast_stowed = False
                        self.sampler.chain_stowed = False
                    else:
                        self.transition_to(t_now, self.state.DEPLOYED)
            # If it has been unsafe to deploy for to long, return to stowed mode
            elif t_now > self.deploy_timeout_time:
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
                
                target_chain_vel = sampler_input.chain * self.sampler.MAX_CHAIN_ROT
                self.sampler.set_chain_vel_trajectory(t_now, self.sampler.CHAIN_RAMP_TIME, [target_chain_vel]*2)



        self.sampler.update(t_now, get_feedback=False)
        
        for handler in self._update_handlers:
            handler(self)
        
    def transition_to(self, t_now: float, state: CoreSamplerControlState):
        if state == self.state:
            return
        
        if state is self.state.DEPLOYING:
            print(self.namespace + "PAYLOAD TRANSITIONING TO DEPLOYING")
            self.deploy_timeout_time = t_now + self.sampler.MAX_DEPLOY_WAIT_TIME
            self.sampler.base_stow_safe = False
        
        if state is self.state.DEPLOYED:
            print(self.namespace + "PAYLOAD TRANSITIONING TO DEPLOYED")

        if state is self.state.STOWING:
            print(self.namespace + "PAYLOAD TRANSITIONING TO STOWING")
            self.sampler.wiggle = False
            self.stow_timeout_time = t_now + self.sampler.MAX_STOW_WAIT_TIME
        
        if state is self.state.STOWED:
            print(self.namespace + "PAYLOAD TRANSITIONING TO STOWED")
            self.sampler.base_stow_safe = True
        
        if state is self.state.DISCONNECTED:
            print(self.namespace + "PAYLOAD TRANSITIONING TO DISCONNECTED")
            self.sampler.mast_traj = None
            self.sampler.chain_traj = None
            self.sampler.wiggle = False
        
        if state is self.state.EMERGENCY_STOP:
            print(self.namespace + "PAYLOAD TRANSITIONING TO EMERGENCY_STOP")
            self.sampler.mast_traj = None
            self.sampler.chain_traj = None
            self.sampler.wiggle = False
        
        if state is self.state.EXIT:
            print(self.namespace + "PAYLOAD TRANSITIONING TO EXIT")
        
        for handler in self._transition_handlers:
            handler(self, state)

        self.state = state

    def stop(self):
        self.transition_to(time(), self.state.EXIT)  