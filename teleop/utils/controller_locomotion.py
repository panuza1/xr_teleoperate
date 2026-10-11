import time
import math


CONTROLLER_INPUT_TIMEOUT = 0.5
CONTROLLER_VELOCITY_SCALE = 0.3
CONTROLLER_YAW_SCALE = 0.3
LOCOMOTION_INPUTS = ("quest", "unitree", "none")
DUET_VX_MAX = 0.4
DUET_VY_MAX = 0.3
DUET_WZ_MAX = 0.6
DUET_STAND_HEIGHT = 0.73


def locomotion_owner(source, motion, arm):
    if source not in LOCOMOTION_INPUTS:
        raise ValueError(f"unsupported locomotion source: {source}")
    return source if motion and arm.startswith("G1") and source != "none" else None


def controller_locomotion_enabled(motion, arm, source="quest"):
    """Return whether Quest owns locomotion for this run."""
    return locomotion_owner(source, motion, arm) == "quest"


def controller_input_fresh(tele_data, now=None, side="right"):
    updated_at = getattr(tele_data, f"{side}_controller_data_updated_at")
    age = (time.monotonic() if now is None else now) - updated_at
    return 0.0 <= age < CONTROLLER_INPUT_TIMEOUT


def controller_tracking_fresh(tele_data, now=None):
    """Return whether both controller pose streams are fresh enough for arm IK."""
    now = time.monotonic() if now is None else now
    return (
        controller_input_fresh(tele_data, now, "left")
        and controller_input_fresh(tele_data, now, "right")
    )


def controller_velocity(tele_data, now=None):
    now = time.monotonic() if now is None else now
    left_fresh = controller_input_fresh(tele_data, now, "left")
    right_fresh = controller_input_fresh(tele_data, now, "right")
    left = tele_data.left_ctrl_thumbstickValue
    right = tele_data.right_ctrl_thumbstickValue
    return (
        -left[1] * CONTROLLER_VELOCITY_SCALE if left_fresh else 0.0,
        -left[0] * CONTROLLER_VELOCITY_SCALE if left_fresh else 0.0,
        -right[0] * CONTROLLER_YAW_SCALE if right_fresh else 0.0,
    )


def fill_duet_wireless_controller(message, tele_data, now=None):
    """Fill Unitree WirelessController fields using DUET keyboard.py conventions."""
    now = time.monotonic() if now is None else now

    def axis(value):
        value = float(value)
        return max(-1.0, min(1.0, value)) if math.isfinite(value) else 0.0

    left = tele_data.left_ctrl_thumbstickValue
    right = tele_data.right_ctrl_thumbstickValue
    left_fresh = controller_input_fresh(tele_data, now, "left")
    right_fresh = controller_input_fresh(tele_data, now, "right")
    message.lx = axis(left[0]) * DUET_VY_MAX if left_fresh else 0.0
    message.ly = -axis(left[1]) * DUET_VX_MAX if left_fresh else 0.0
    message.rx = axis(right[0]) * DUET_WZ_MAX if right_fresh else 0.0
    message.ry = DUET_STAND_HEIGHT
    message.keys = 0
    return message


def apply_controller_locomotion(tele_data, loco_wrapper, now=None, source="quest"):
    if source != "quest":
        return False
    now = time.monotonic() if now is None else now
    left_fresh = controller_input_fresh(tele_data, now, "left")
    right_fresh = controller_input_fresh(tele_data, now, "right")
    if left_fresh and right_fresh and tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
        loco_wrapper.Damp()
    else:
        loco_wrapper.Move(*controller_velocity(tele_data, now))
    return right_fresh and tele_data.right_ctrl_aButton
