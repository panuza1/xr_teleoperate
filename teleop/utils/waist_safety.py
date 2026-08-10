import numpy as np


# Conservative first-hardware-test limits. Widening these values must be an
# explicit tuning decision; they are intentionally well inside the G1 limits.
WAIST_LIMITS_RAD = np.deg2rad(np.array([25.0, 12.0, 12.0], dtype=np.float64))

# Maximum commanded change at each 250 Hz DDS cycle (yaw, roll, pitch).
WAIST_MAX_DELTA_PER_CYCLE_RAD = np.array([0.002, 0.0015, 0.0015], dtype=np.float64)

WAIST_HOME_RAMP_RAD_S = 0.25


def _waist_vector(values, name):
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {vector.shape}")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain only finite values")
    return vector


def clamp_waist_target(target):
    """Clamp a yaw/roll/pitch target to the conservative software limits."""
    target = _waist_vector(target, "waist target")
    return np.clip(target, -WAIST_LIMITS_RAD, WAIST_LIMITS_RAD)


def rate_limit_waist_target(target, previous, max_delta=WAIST_MAX_DELTA_PER_CYCLE_RAD):
    """Clamp a waist target, then bound its change from the previous DDS command."""
    target = clamp_waist_target(target)
    previous = _waist_vector(previous, "previous waist command")
    max_delta = _waist_vector(max_delta, "waist max delta")
    if np.any(max_delta <= 0.0):
        raise ValueError("waist max delta must be positive")
    return previous + np.clip(target - previous, -max_delta, max_delta)


def body_tracking_ok(body_tracking_ready, body_pose_updated_at, now, tracking_timeout):
    """Return whether at least one body frame arrived and the latest one is fresh."""
    timing = np.asarray([body_pose_updated_at, now, tracking_timeout], dtype=np.float64)
    if not np.all(np.isfinite(timing)):
        return False
    if not body_tracking_ready or body_pose_updated_at <= 0.0 or tracking_timeout <= 0.0:
        return False
    return (now - body_pose_updated_at) <= tracking_timeout


def rate_limit_sdk_weight(current, target, max_delta):
    """Move the shared arm_sdk blend weight by one bounded control-cycle step."""
    values = np.asarray([current, target, max_delta], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("SDK weight values must be finite")
    if max_delta <= 0.0:
        raise ValueError("SDK weight max delta must be positive")
    current = float(np.clip(current, 0.0, 1.0))
    target = float(np.clip(target, 0.0, 1.0))
    return current + float(np.clip(target - current, -max_delta, max_delta))


def resolve_waist_fallback(last_target, tracking_fallback, frequency, home_ramp_rad_s=WAIST_HOME_RAMP_RAD_S):
    """Apply the configured hold/home policy to the last waist target."""
    target = clamp_waist_target(last_target)
    if tracking_fallback == "hold":
        return target
    if tracking_fallback != "home":
        raise ValueError(f"unknown waist tracking fallback: {tracking_fallback}")
    if not np.isfinite(frequency) or frequency <= 0.0:
        raise ValueError("frequency must be positive")
    if not np.isfinite(home_ramp_rad_s) or home_ramp_rad_s <= 0.0:
        raise ValueError("home ramp speed must be positive")
    max_step = float(home_ramp_rad_s) / float(frequency)
    return target + np.clip(-target, -max_step, max_step)
