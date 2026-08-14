import time


CONTROLLER_INPUT_TIMEOUT = 0.5
CONTROLLER_VELOCITY_SCALE = 0.3


def controller_locomotion_enabled(motion, arm):
    return motion and arm.startswith("G1")


def controller_input_fresh(tele_data, now=None):
    age = (time.monotonic() if now is None else now) - tele_data.controller_data_updated_at
    return 0.0 <= age < CONTROLLER_INPUT_TIMEOUT


def controller_velocity(tele_data, now=None):
    if not controller_input_fresh(tele_data, now):
        return 0.0, 0.0, 0.0
    left = tele_data.left_ctrl_thumbstickValue
    right = tele_data.right_ctrl_thumbstickValue
    return (
        -left[1] * CONTROLLER_VELOCITY_SCALE,
        -left[0] * CONTROLLER_VELOCITY_SCALE,
        -right[0] * CONTROLLER_VELOCITY_SCALE,
    )


def apply_controller_locomotion(tele_data, loco_wrapper, now=None):
    fresh = controller_input_fresh(tele_data, now)
    if fresh and tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
        loco_wrapper.Damp()
    else:
        loco_wrapper.Move(*controller_velocity(tele_data, now))
    return fresh and tele_data.right_ctrl_aButton
