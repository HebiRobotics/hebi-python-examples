"""Arm controllers in kits/arms (and Rosie's arm setup), run against imitated arms."""

import time
from pathlib import Path

import hebi
import numpy as np
import pytest

from conftest import REPO_ROOT, unique_family

ARMS_CONFIG = REPO_ROOT / "kits" / "arms" / "config"
HOME = [0.0, 1.2, 1.8, 2.2, -1.57, 0.0]


def imitated_arm(imitation, lookup, cfg_path: Path, extra_names=(), tag="arm"):
    """Load an arm config, re-family it to a unique pytest family, imitate it, and create the arm."""
    cfg = hebi.config.load_config(str(cfg_path))
    family = unique_family(tag)
    imitation([(family, name) for name in [*cfg.names, *extra_names]])
    cfg.families = [family]
    return hebi.arm.create_from_config(cfg, lookup), cfg


def run_until(predicate, step, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        step(time.time())
        if predicate():
            return True
    return False


# ---------------------------------------------------------------- joystick_control_sm


def test_joystick_teleop_sets_goal(imitation, lookup):
    """A1: teleop goals are built with add_waypoint(position=...) and no `t`, which raises on hebi >= 2.16.1."""
    from kits.arms.joystick_control_sm import ArmControlState, ArmJoystickControl, ArmJoystickInputs

    arm, _ = imitated_arm(imitation, lookup, ARMS_CONFIG / "A-2580-06.cfg.yaml")
    ctrl = ArmJoystickControl(arm, HOME, homing_time=0.3)
    idle = ArmJoystickInputs(False, [0, 0, 0], [0, 0, 0], False)

    def step(t):
        ctrl.update(t, idle)
        ctrl.send()

    assert run_until(lambda: ctrl.state is ArmControlState.TELEOP, step), ctrl.state
    ctrl.update(time.time(), ArmJoystickInputs(False, [0.02, 0, 0], [0, 0, 0], False))
    assert arm.trajectory is not None


def test_joystick_joint_target_is_a_copy(imitation, lookup):
    """A2: joint_target must not alias the feedback buffer (it's modified in place for the shoulder flip)."""
    from kits.arms.joystick_control_sm import ArmControlState, ArmJoystickControl, ArmJoystickInputs

    arm, _ = imitated_arm(imitation, lookup, ARMS_CONFIG / "A-2580-06.cfg.yaml")
    arm.update()
    ctrl = ArmJoystickControl(arm, HOME, homing_time=0.3)
    assert not np.shares_memory(ctrl.joint_target, arm.last_feedback.position_command)
    assert not np.any(np.isnan(ctrl.joint_target))

    idle = ArmJoystickInputs(False, [0, 0, 0], [0, 0, 0], False)

    def step(t):
        ctrl.update(t, idle)
        ctrl.send()

    assert run_until(lambda: ctrl.state is ArmControlState.TELEOP, step)
    assert not np.shares_memory(ctrl.joint_target, arm.last_feedback.position_command)


# ---------------------------------------------------------------- ar_control_sm


def test_ar_teleop_home_button_homes(imitation, lookup):
    """A3: the home button is ignored while tracking (TELEOP)."""
    from kits.arms.ar_control_sm import ArmControlState, ArmMobileIOControl, ArmMobileIOInputs

    arm, _ = imitated_arm(imitation, lookup, ARMS_CONFIG / "A-2580-06.cfg.yaml")
    ctrl = ArmMobileIOControl(arm, None, homing_time=0.3)

    def inputs(**kw):
        return ArmMobileIOInputs(phone_pos=np.zeros(3), phone_rot=np.eye(3), **kw)

    def step(t):
        ctrl.update(t, inputs())
        ctrl.send()

    assert run_until(lambda: ctrl.state is ArmControlState.IDLE, step), ctrl.state
    ctrl.update(time.time(), inputs(lock_toggle=True, locked=False))
    assert ctrl.state is ArmControlState.TELEOP
    ctrl.update(time.time(), inputs(locked=False, home=True))
    assert ctrl.state is ArmControlState.HOMING


def test_ar_main_parser_buttons(fake_mobile_io):
    """A4: the standalone AR demo read the lock button's state as the toggle, and the grip button as the lock."""
    from kits.arms.ar_control_sm import parse_mobile_feedback

    fake_mobile_io.press(b5=1)  # unlock (toggle on)
    _, inputs = parse_mobile_feedback(fake_mobile_io)
    assert inputs.lock_toggle and not inputs.locked
    assert not inputs.gripper_closed

    fake_mobile_io.press(b5=1, b7=1)  # still unlocked, close gripper
    _, inputs = parse_mobile_feedback(fake_mobile_io)
    assert not inputs.lock_toggle and not inputs.locked
    assert inputs.gripper_closed


# ---------------------------------------------------------------- ex_leader_follower


def test_leader_follower_idle_holds(imitation, lookup):
    """A1: Idle re-commands the follower with add_waypoint(position=...) and no `t`.
    Also, the first update with no Mobile IO input read an attribute that was never set."""
    from kits.arms.ex_leader_follower import FollowerControlState, LeaderFollowerControl, LeaderFollowerInputs

    # One config object per arm: create_from_config consumes the config's plugin entries.
    leader_cfg = hebi.config.load_config(str(ARMS_CONFIG / "A-2085-06.cfg.yaml"))
    follower_cfg = hebi.config.load_config(str(ARMS_CONFIG / "A-2085-06.cfg.yaml"))
    leader_family, follower_family = unique_family("leader"), unique_family("follower")
    imitation([(f, n) for f in (leader_family, follower_family) for n in leader_cfg.names])
    leader_cfg.families = [leader_family]
    follower_cfg.families = [follower_family]
    leader = hebi.arm.create_from_config(leader_cfg, lookup)
    follower = hebi.arm.create_from_config(follower_cfg, lookup)

    ctrl = LeaderFollowerControl(leader, follower, home_pose=HOME, homing_time=0.3)
    ctrl.update(time.time(), None)  # no Mobile IO sample yet

    assert run_until(lambda: ctrl.state is FollowerControlState.IDLE, lambda t: ctrl.update(t, LeaderFollowerInputs()))
    ctrl.update(time.time(), LeaderFollowerInputs())
    assert follower.trajectory is not None


# ---------------------------------------------------------------- Rosie


def test_rosie_gripper_efforts_match_config(imitation, lookup):
    """A5: rosie_utils.setup_arm read gripper_open_effort into the close effort and vice versa."""
    from kits.rosie.rosie_utils import setup_arm

    cfg = hebi.config.load_config(str(REPO_ROOT / "kits" / "rosie" / "config" / "rosie-t.cfg.yaml"))
    family = unique_family("rosie")
    imitation([(family, name) for name in [*cfg.names, "gripperSpool"]])
    cfg.families = [family]

    arm, gripper = setup_arm(cfg, lookup)
    assert arm is not None and gripper is not None
    gripper.close()
    assert np.isclose(gripper.command.effort[0], cfg.user_data["gripper_close_effort"])
    gripper.open()
    assert np.isclose(gripper.command.effort[0], cfg.user_data["gripper_open_effort"])
