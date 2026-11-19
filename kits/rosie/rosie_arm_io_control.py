import sys
from time import time, sleep
import os
import datetime
import warnings

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi
from hebi.util import create_mobile_io

import threading

from kits.arms.ar_control_sm import (
    ARHomingState,
    ARIdleState,
    ARTeleopState,
    ArmMobileIOController,
    ArmMobileIOInputs,
)
from kits.arms.arm_controller import ArmExitState
from kits.common.config import add_config_argument, config_arg_parser, load_config
from ..bases.omni_base import ChassisVelocity, OmniBase
from .rosie_utils import set_mobile_io_instructions, setup_base, setup_arm

import typing

if typing.TYPE_CHECKING:
    from hebi._internal.mobile_io import MobileIO


class ArmMobileIOUI:
    """Arm status on the Mobile IO.

    ``on_transition`` runs on the arm's control thread, so it only records the
    message; ``flush()`` sends it from the main thread.
    """

    def __init__(self, mio: "MobileIO"):
        self.mio = mio
        self._lock = threading.Lock()
        self._pending: tuple[str, str | None] | None = None
        self._reset_ui = False

    def on_transition(self, controller: ArmMobileIOController, new_state):
        match new_state:
            case ARHomingState():
                self._post("Robot Homing Sequence\nPlease wait...", None)
            case ARIdleState() | ARTeleopState():
                self._post("Robot Ready to Control", "green")
            case ArmExitState():
                with self._lock:
                    self._reset_ui = True
                self._post("Demo Stopped.", "red")

    def _post(self, msg: str, color: str | None):
        with self._lock:
            self._pending = (msg, color)

    def flush(self):
        with self._lock:
            pending, self._pending = self._pending, None
            reset_ui, self._reset_ui = self._reset_ui, False
        if reset_ui:
            self.mio.resetUI()
        if pending is not None:
            set_mobile_io_instructions(self.mio, *pending)


class RosieControl:
    def __init__(self, base: "OmniBase"):
        self.base = base
        self.running = True
        self.on_shutdown = lambda: None

    def update(self, t_now, base_inputs: "ChassisVelocity | None"):
        self.base.update(t_now)
        if base_inputs is None:
            return
        self.base.build_smooth_velocity_trajectory(base_inputs.x, base_inputs.y, base_inputs.rz, t_now)

    def send(self):
        self.base.send()

    def stop(self):
        self.running = False
        self.base.base_command.velocity = np.zeros(3)
        self.on_shutdown()


def setup_mobile_io(m: "MobileIO"):
    """Sets up mobileIO interface.

    Return a function that parses mobileIO feedback into the format
    expected by the Demo
    """

    # Base
    reset_pose_btn = 1
    quit_demo_btn = 8

    side_joy = 1  # Left Pad Left/Right
    forward_joy = 2  # Left Pad Up/Down
    ar_xyz_scale_slider = 4
    turn_joy = 7  # Right Pad Left/Right

    m.resetUI()

    m.set_button_label(reset_pose_btn, "⟲", blocking=False)
    m.set_button_label(quit_demo_btn, "❌", blocking=False)

    m.set_axis_label(side_joy, "", blocking=False)
    m.set_axis_label(forward_joy, "Translate", blocking=False)
    m.set_axis_label(ar_xyz_scale_slider, "XYZ\nScale", blocking=False)
    m.set_axis_label(turn_joy, "", blocking=False)
    m.set_axis_label(8, "Rotate", blocking=False)

    for i, snap in enumerate([0, 0, 0, np.nan, 0, 0, 0, 0]):
        m.set_snap(i + 1, snap)

    global ar_scaling
    ar_scaling = 1.0
    m.set_axis_value(ar_xyz_scale_slider, ar_scaling)

    m.set_led_color("yellow")

    # Arm
    arm_lock = 3
    gripper_close = 4

    m.set_button_label(arm_lock, "Arm 🔒", blocking=False)
    m.set_button_label(gripper_close, "Gripper", blocking=False)
    m.set_button_mode(arm_lock, 1)
    m.set_button_mode(gripper_close, 1)
    m.set_button_output(arm_lock, 1)
    m.set_button_output(gripper_close, 1)

    def parse_mobile_io_feedback(m: "MobileIO"):
        if not m.update(0.0):
            return False, None, None
        else:
            if m.get_button_state(quit_demo_btn):
                return True, None, None

            user_reset = m.get_button_state(reset_pose_btn)

            chassis_velocity = ChassisVelocity(
                pow(m.get_axis_state(forward_joy), 3),
                pow(-m.get_axis_state(side_joy), 3),
                pow(-m.get_axis_state(turn_joy), 3) * 2,
            )

            try:
                wxyz = m.orientation
                xyzw = [*wxyz[1:], wxyz[0]]
                rotation = R.from_quat(xyzw).as_matrix()
            except ValueError as e:
                print(f"Error getting orientation as matrix: {e}\n{m.orientation}")
                # Invalid pose: the arm holds position rather than tracking a made-up orientation
                rotation = np.full((3, 3), np.nan)

            if m.get_button_diff(arm_lock) != 0:
                if not m.get_button_state(arm_lock):
                    m.set_button_label(arm_lock, "Arm 🔒", blocking=False)
                elif m.get_button_state(arm_lock):
                    m.set_button_label(arm_lock, "Arm 🔓", blocking=False)

            locked = m.get_button_state(arm_lock)
            global ar_scaling
            if locked:
                m.set_axis_value(ar_xyz_scale_slider, (2 * ar_scaling) - 1.0)
            if not locked:
                ar_scaling = (m.get_axis_state(ar_xyz_scale_slider) + 1.0) / 2.0
                if ar_scaling < 0.1:
                    ar_scaling = 0.0

            arm_inputs = ArmMobileIOInputs(
                phone_pos=np.copy(m.position),
                phone_rot=rotation,
                ar_scaling=ar_scaling,
                lock_toggle=(m.get_button_diff(arm_lock) != 0),
                locked=(not locked),
                gripper_closed=m.get_button_state(gripper_close),
                home=user_reset,
            )

            return False, chassis_velocity, arm_inputs

    return parse_mobile_io_feedback


if __name__ == "__main__":
    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
    parser = config_arg_parser("Rosie: omni base + AR-controlled arm", config_dir)
    add_config_argument(parser, "config", "Rosie config, e.g. rosie-t.cfg.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config, config_dir)
    family = cfg.families[0]
    lookup = hebi.Lookup()

    # mobileIO setup
    print("Waiting for mobileIO device to come online...")
    while (m := create_mobile_io(lookup, family)) is None:
        print("Could not find mobileIO device, waiting...")
        sleep(0.5)

    print("mobileIO device found.")
    parse_mobile_feedback = setup_mobile_io(m)

    # Base setup
    base = setup_base(lookup, family)
    base_control = RosieControl(base)

    tries = 3
    arm, gripper = setup_arm(cfg, lookup)
    while arm is None and tries > 0:
        sleep(1)
        tries -= 1
        arm, gripper = setup_arm(cfg, lookup)

    arm_ui = ArmMobileIOUI(m)
    if arm is None:
        warnings.warn("Could not find arm! Continuing without...")
        arm_control = None
        set_mobile_io_instructions(m, "Robot Ready to Control", color="green")

        def on_shutdown():
            if m:
                set_mobile_io_instructions(m, "Demo Stopped.", color="red")

        base_control.on_shutdown = on_shutdown
    else:
        while not arm.update():
            print("Waiting for feedback from arm...")
            sleep(1.0)

        arm_control = ArmMobileIOController.from_config(arm, cfg, gripper)
        arm_control.add_transition_handler(arm_ui.on_transition)

    #######################
    ## Main Control Loop ##
    #######################

    logging = True
    rosie_dir = os.path.dirname(__file__)
    log_dir = os.path.join(rosie_dir, "logs")

    if logging:
        now = datetime.datetime.now()
        base.group.start_log(log_dir, f"base_{now:%Y-%m-%d_%H-%M-%S}", mkdirs=True)
        if arm_control:
            arm_control.start_logging(log_dir, "arm")

    if arm_control:
        arm_control.start()

    try:
        while base_control.running and (arm_control is None or arm_control.running):
            t = time()
            quit, base_inputs, arm_inputs = parse_mobile_feedback(m)
            if quit:
                break
            base_control.update(t, base_inputs)
            base_control.send()
            if arm_control and arm_inputs is not None:
                arm_control.receive_inputs(arm_inputs)
            arm_ui.flush()
    except KeyboardInterrupt:
        pass
    finally:
        base_control.stop()
        base_control.send()
        if arm_control:
            arm_control.stop()  # also stops the arm log
        arm_ui.flush()
        if logging:
            base.group.stop_log()

    sys.exit(0)
