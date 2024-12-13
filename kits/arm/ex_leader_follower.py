import os
from enum import Enum, auto
import numpy as np
from time import time, sleep

import hebi
from hebi.util import create_mobile_io

import typing

if typing.TYPE_CHECKING:
    from typing import Sequence, Optional
    import numpy.typing as npt
    from hebi._internal.mobile_io import MobileIO


class FollowerControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    ALIGNING = auto()
    IDLE = auto()
    FOLLOW = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class LeaderFollowerInputs:
    def __init__(
        self, home: bool = False, follow: bool = False, haptic_fbk: bool = False
    ):
        self.home = home
        self.follow = follow
        self.haptic_fbk = haptic_fbk


class LeaderFollowerControl:
    def __init__(
        self,
        leader_arm: hebi.arm.Arm,
        follower_arm: hebi.arm.Arm,
        home_pose: "Sequence[float] | npt.NDArray[np.float64]",
        homing_time: float = 3.0,
        haptic_gains: "Optional[Sequence[float]]" = np.array([30, 25, 20, 10, 3, 2]) * 10,
        haptic_limit: float = 100
    ):
        self.namespace = ""

        self.state = FollowerControlState.STARTUP
        self.leader_arm = leader_arm
        self.follower_arm = follower_arm

        self.arm_home = home_pose
        self.homing_time = homing_time

        self.haptic_enabled = False

        self.haptic_gains = haptic_gains
        self.haptic_limit = haptic_limit

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.leader_arm.send()
        self.follower_arm.send()

    def update(self, t_now: float, demo_input: "Optional[LeaderFollowerInputs]" = None):
        self.leader_arm.update()
        self.follower_arm.update()

        if self.state is self.state.EXIT:
            return

        if demo_input is None:
            if t_now - self.mobile_last_fbk_t > 1.0 and self.state is not self.state.DISCONNECTED:
                print(self.namespace + "mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
        else:
            self.mobile_last_fbk_t = t_now

        if self.state is self.state.DISCONNECTED:
            self.transition_to(t_now, self.state.IDLE)

        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

        elif self.state is self.state.HOMING:
            if self.follower_arm.at_goal:
                self.transition_to(t_now, self.state.IDLE)

        elif self.state is self.state.ALIGNING:
            if self.follower_arm.at_goal:
                self.transition_to(t_now, self.state.FOLLOW)

        elif self.state is self.state.IDLE:
            if demo_input and demo_input.home:
                self.transition_to(t_now, self.state.HOMING)
                return
            elif demo_input and demo_input.follow:
                self.transition_to(t_now, self.state.ALIGNING)
                return

            arm_goal = hebi.arm.Goal(self.follower_arm.size)
            pos_curr = self.follower_arm.last_feedback.position_command
            if np.any(np.isnan(pos_curr)):
                print(self.namespace + "No position command, falling back to feedback position")
                pos_curr = self.follower_arm.last_feedback.position
            arm_goal.add_waypoint(position=pos_curr)
            self.follower_arm.set_goal(arm_goal)

        elif self.state is self.state.FOLLOW:
            if demo_input and demo_input.home:
                self.transition_to(t_now, self.state.HOMING)
                return
            elif demo_input and not demo_input.follow:
                self.transition_to(t_now, self.state.IDLE)
                return
            if demo_input:
                self.haptic_enabled = demo_input.haptic_fbk

            self.follower_arm.pending_command.position = self.leader_arm.last_feedback.position
            self.follower_arm.pending_command.velocity = self.leader_arm.last_feedback.velocity

            follower_double_shoulder = self.follower_arm.get_plugin_by_type(hebi.arm.DoubledJointMirror)
            if follower_double_shoulder:
                follower_double_shoulder.update(self.follower_arm, 0)
 
            if self.haptic_enabled:
                pos_diff = self.leader_arm.last_feedback.position - self.follower_arm.last_feedback.position
                haptic_effort = self.haptic_gains * pos_diff * np.abs(pos_diff)
                haptic_effort = np.clip(haptic_effort, -self.haptic_limit, self.haptic_limit)
                if np.any(np.isnan(haptic_effort)):
                    self.leader_arm.pending_command.effort = np.zeros(self.leader_arm.size)
                self.leader_arm.pending_command.effort = self.leader_arm.pending_command.effort - haptic_effort

        self.send()

    def transition_to(self, t_now: float, state: FollowerControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print(self.namespace + "TRANSITIONING TO HOMING")
            g = hebi.arm.Goal(self.follower_arm.size)
            g.add_waypoint(t=self.homing_time, position=self.arm_home)
            self.follower_arm.set_goal(g)

        elif state is self.state.ALIGNING:
            print(self.namespace + "TRANSITIONING TO ALIGNING")
            g = hebi.arm.Goal(self.follower_arm.size)
            g.add_waypoint(t=self.homing_time, position=self.leader_arm.last_feedback.position)
            self.follower_arm.set_goal(g)

        elif state is self.state.IDLE:
            print(self.namespace + "TRANSITIONING TO IDLE")

        elif state is self.state.FOLLOW:
            self.leader_arm.cancel_goal()
            self.leader_arm.get_plugin_by_type(hebi.arm.DoubledJointMirror)._cmd.position = np.nan 
            print(self.namespace + "TRANSITIONING TO FOLLOW")

        elif state is self.state.DISCONNECTED:
            print(self.namespace + "TRANSITIONING TO DISCONNECTED")

        elif state is self.state.EXIT:
            print(self.namespace + "TRANSITIONING TO EXIT")

        self.state = state
        self.haptic_enabled = False
    
    def stop(self):
        self.transition_to(time(), FollowerControlState.EXIT)


def setup_mobile_io(m: "MobileIO"):
    m.resetUI()
    m.set_button_label(1, "⟲")

    m.set_button_label(2, "Follow")
    m.set_button_mode(2, 1)

    m.set_button_label(3, "Haptic")
    m.set_button_mode(3, 1)

    m.set_button_label(4, " ")
    m.set_button_label(5, " ")
    m.set_button_label(6, " ")
    m.set_button_label(7, " ")
    m.set_button_label(8, "Quit")

    m.set_axis_label(1, " ")
    m.set_axis_label(2, " ")
    m.set_axis_label(3, " ")
    m.set_axis_label(4, " ")
    m.set_axis_label(5, " ")
    m.set_axis_label(6, " ")
    m.set_axis_label(7, " ")
    m.set_axis_label(8, " ")


def parse_mobile_feedback(m: "MobileIO"):
    if not m.update(0.0):
        return None

    home = m.get_button_state(1)
    follow = m.get_button_state(2) == 1
    haptic_fbk = m.get_button_state(3) == 1

    return LeaderFollowerInputs(home, follow, haptic_fbk)


if __name__ == "__main__":
    root_dir = os.path.abspath(os.path.dirname(__file__))
    lookup = hebi.Lookup()
    sleep(2)

    # Leader Arm setup
    leader_arm_family = "Leader-Arm"
    leader_module_names = [
        "J1_base",
        "J2_shoulder",
        "J3_elbow",
        "J4_wrist1",
        "J5_wrist2",
        "J6_wrist3",
    ]
    leader_hrdf_file = "config/hrdf/A-2085-06.hrdf"
    leader_gains_file = "config/gains/A-2085-06.xml"

    leader_hrdf_file = os.path.join(root_dir, leader_hrdf_file)
    leader_gains_file = os.path.join(root_dir, leader_gains_file)

    leader_arm = hebi.arm.create(
        [leader_arm_family],
        names=leader_module_names,
        hrdf_file=leader_hrdf_file,
        lookup=lookup,
    )
    leader_arm.load_gains(leader_gains_file)

    # Follower Arm setup
    follower_arm_family = "Follower-Arm"
    follower_module_names = [
        "J1_base",
        "J2_shoulder",
        "J3_elbow",
        "J4_wrist1",
        "J5_wrist2",
        "J6_wrist3",
    ]
    follower_hrdf_file = "config/hrdf/A-2085-06.hrdf"
    follower_gains_file = "config/gains/A-2085-06.xml"

    follower_hrdf_file = os.path.join(root_dir, follower_hrdf_file)
    follower_gains_file = os.path.join(root_dir, follower_gains_file)

    follower_arm = hebi.arm.create(
        [follower_arm_family],
        names=follower_module_names,
        hrdf_file=follower_hrdf_file,
        lookup=lookup,
    )
    follower_arm.load_gains(follower_gains_file)

    # Setup LeaderFollowerControl
    leader_follower_control = LeaderFollowerControl(
        leader_arm, follower_arm, home_pose=[0.0, 2.09, 2.09, 0.0, 1.57, 0.0]
    )

    # Setup MobileIO
    print("Looking for Mobile IO...")
    m = create_mobile_io(lookup, "Arm")
    while m is None:
        try:
            print("Waiting for Mobile IO device to come online...")
            sleep(1)
            m = create_mobile_io(lookup, "Arm")
        except KeyboardInterrupt:
            leader_follower_control.stop()
            exit()

    m.update()
    setup_mobile_io(m)

    #######################
    ## Main Control Loop ##
    #######################

    while leader_follower_control.running:
        t = time()
        try:
            arm_inputs = parse_mobile_feedback(m)
            leader_follower_control.update(t, arm_inputs)

            leader_follower_control.send()
        except KeyboardInterrupt:
            leader_follower_control.stop()
            m.set_led_color("red")

        if m.get_button_state(8):
            leader_follower_control.stop()
            m.set_led_color("red")
