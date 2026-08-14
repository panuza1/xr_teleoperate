import time


CONTROLLER_INPUT_TIMEOUT = 0.5
CONTROLLER_VELOCITY_SCALE = 0.3
CONTROLLER_YAW_SCALE = 0.15


def controller_locomotion_enabled(motion, arm):
    return motion and arm.startswith("G1")


def controller_input_fresh(tele_data, now=None, side="right"):
    updated_at = getattr(tele_data, f"{side}_controller_data_updated_at")
    age = (time.monotonic() if now is None else now) - updated_at
    return 0.0 <= age < CONTROLLER_INPUT_TIMEOUT


def controller_velocity(tele_data, now=None):
    now = time.monotonic() if now is None else now
    left_fresh = controller_input_fresh(tele_data, now, "left")
    right_fresh = controller_input_fresh(tele_data, now, "right")
    left = tele_data.left_ctrl_thumbstickValue
    right = tele_data.right_ctrl_thumbstickValue
    return (
        -right[1] * CONTROLLER_VELOCITY_SCALE if right_fresh else 0.0,
        -right[0] * CONTROLLER_VELOCITY_SCALE if right_fresh else 0.0,
        -left[0] * CONTROLLER_YAW_SCALE if left_fresh else 0.0,
    )


def apply_controller_locomotion(tele_data, loco_wrapper, now=None):
    now = time.monotonic() if now is None else now
    left_fresh = controller_input_fresh(tele_data, now, "left")
    right_fresh = controller_input_fresh(tele_data, now, "right")
    if left_fresh and right_fresh and tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
        loco_wrapper.Damp()
    else:
        loco_wrapper.Move(*controller_velocity(tele_data, now))
    return right_fresh and tele_data.right_ctrl_aButton
