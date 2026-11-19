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


# The arm controllers run their own control thread (Controller.start()/stop()). Imitated modules report
# the M-stop as pressed, so tests drive it with this flag instead.
@pytest.fixture(autouse=True)
def no_mstop(monkeypatch):
    monkeypatch.setattr(hebi.arm.Arm, "m_stopped", property(lambda self: getattr(self, "test_mstop", False)))


def wait_for(ctrl, cls, timeout_s=5.0, feed=None):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not isinstance(ctrl.state, cls):
        if feed is not None:
            ctrl.receive_inputs(feed())
        time.sleep(0.02)
    return isinstance(ctrl.state, cls)


@pytest.fixture
def started():
    """Stops every controller passed to it at the end of the test."""
    controllers = []

    def start(ctrl):
        controllers.append(ctrl)
        ctrl.start()
        return ctrl

    yield start
    for ctrl in controllers:
        if ctrl.running:
            ctrl.stop()


# ---------------------------------------------------------------- joystick_control_sm


def test_joystick_jog_sets_goal(imitation, lookup, started):
    """add_waypoint(position=...) without `t` raised on hebi >= 2.16.1; joint_target aliased the feedback buffer."""
    from kits.arms import joystick_control_sm as js

    arm, cfg = imitated_arm(imitation, lookup, ARMS_CONFIG / "A-2580-06.cfg.yaml")
    ctrl = started(js.ArmJoystickController.from_config(arm, cfg))
    assert wait_for(ctrl, js.JoystickTeleopState, feed=js.ArmJoystickInputs.idle)
    target = ctrl.state.joint_target
    assert not np.shares_memory(target, arm.last_feedback.position_command) and not np.any(np.isnan(target))

    ctrl.receive_inputs(js.ArmJoystickInputs(home=False, delta_xyz=[0.02, 0, 0], delta_rot_xyz=[0, 0, 0]))
    assert arm.trajectory is not None


def test_joystick_disconnect_holds_position(imitation, lookup, started):
    """On Mobile IO disconnect the arm was sent to its home pose in 0.1 s."""
    from kits.arms import joystick_control_sm as js
    from kits.common.state_machine import DisconnectedState

    arm, cfg = imitated_arm(imitation, lookup, ARMS_CONFIG / "A-2580-06.cfg.yaml")
    ctrl = started(js.ArmJoystickController.from_config(arm, cfg))
    assert wait_for(ctrl, js.JoystickTeleopState, feed=js.ArmJoystickInputs.idle)
    ctrl.receive_inputs(js.ArmJoystickInputs(home=False, delta_xyz=[0.05, 0, 0], delta_rot_xyz=[0, 0, 0]))
    assert wait_for(ctrl, DisconnectedState, timeout_s=3)
    traj = arm.trajectory
    assert np.allclose(traj.get_state(traj.end_time)[1], 0.0)
    assert not np.allclose(traj.get_state(traj.end_time)[0], ctrl.arm_home, atol=1e-3), "moved to home on disconnect"


# ---------------------------------------------------------------- ar_control_sm


def test_ar_teleop_home_button_homes(imitation, lookup, started):
    """The home button was ignored while tracking (Teleop)."""
    from kits.arms import ar_control_sm as ar

    arm, cfg = imitated_arm(imitation, lookup, ARMS_CONFIG / "A-2580-06.cfg.yaml")
    ctrl = started(ar.ArmMobileIOController.from_config(arm, cfg))

    def inputs(**kw):
        return ar.ArmMobileIOInputs(phone_pos=np.zeros(3), phone_rot=np.eye(3), **kw)

    assert wait_for(ctrl, ar.ARIdleState, feed=inputs)
    # An input only records a request; the transition happens on the next control tick.
    assert wait_for(ctrl, ar.ARTeleopState, feed=lambda: inputs(lock_toggle=True, locked=False))
    assert wait_for(ctrl, ar.ARHomingState, feed=lambda: inputs(locked=False, home=True))


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


def test_ar_invalid_phone_orientation_is_nan(fake_mobile_io):
    """An invalid orientation must hold the arm (NaN pose is skipped), not track a made-up identity pose."""
    from kits.arms.ar_control_sm import phone_rotation

    fake_mobile_io.orientation = [0.0, 0.0, 0.0, 0.0]  # zero-norm quaternion: scipy raises ValueError
    assert np.all(np.isnan(phone_rotation(fake_mobile_io)))


def test_joystick_parser_treats_nan_axes_as_no_input(fake_mobile_io):
    """Axes can read NaN right after a layout is sent, before the phone reports them."""
    from kits.tready.treaded_base_arm_joystick_control import parse_joystick_arm

    for axis in fake_mobile_io.axes:
        fake_mobile_io.axes[axis] = float("nan")
    fake_mobile_io.press(b3=1)  # arm mode on: also reads the rotation axes
    inputs = parse_joystick_arm(fake_mobile_io)
    assert np.all(np.asarray(inputs.delta_xyz) == 0.0)
    assert np.all(np.asarray(inputs.delta_rot_xyz) == 0.0)


def test_arm_led_colors_are_the_same_in_every_demo():
    """Homing is magenta, disconnected is blue, everything else is transparent."""
    from types import SimpleNamespace

    from kits.arms.ar_control_sm import ARHomingState, ARIdleState
    from kits.arms.arm_controller import ArmController
    from kits.arms.joystick_control_sm import JoystickHomingState, JoystickTeleopState
    from kits.common.state_machine import DisconnectedState

    led = SimpleNamespace(color="")
    controller = SimpleNamespace(arm=SimpleNamespace(pending_command=SimpleNamespace(led=led)))

    for state_cls, color in [
        (ARHomingState, "magenta"),
        (JoystickHomingState, "magenta"),
        (DisconnectedState, "blue"),
        (ARIdleState, "transparent"),
        (JoystickTeleopState, "transparent"),
    ]:
        ArmController._set_led_color(controller, object.__new__(state_cls))
        assert led.color == color, state_cls.__name__


# ---------------------------------------------------------------- ex_leader_follower


def test_leader_follower_idle_holds(imitation, lookup, started):
    """Idle held the follower with add_waypoint(position=...) and no `t`; reusing one config object for both
    arms raised KeyError (create_from_config consumes its plugin entries)."""
    from kits.arms import ex_leader_follower as lf

    leader_cfg = hebi.config.load_config(str(ARMS_CONFIG / "ex_leader_follower_leader.cfg.yaml"))
    follower_cfg = hebi.config.load_config(str(ARMS_CONFIG / "ex_leader_follower.cfg.yaml"))
    leader_family, follower_family = unique_family("leader"), unique_family("follower")
    imitation([(f, n) for f in (leader_family, follower_family) for n in leader_cfg.names])
    leader_cfg.families = [leader_family]
    follower_cfg.families = [follower_family]
    leader = hebi.arm.create_from_config(leader_cfg, lookup)
    follower = hebi.arm.create_from_config(follower_cfg, lookup)

    ctrl = started(lf.LeaderFollowerController(follower, leader, home_position=follower_cfg.user_data["home_position"]))
    assert wait_for(ctrl, lf.LFIdleState, feed=lf.LeaderFollowerInputs)
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
