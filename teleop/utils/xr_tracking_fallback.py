import numpy as np


def xr_arm_poses_valid(raw_left_pose, raw_right_pose):
    """Return True when both raw XR arm poses are finite non-singular SE(3) matrices."""
    raw_left_pose = np.asarray(raw_left_pose, dtype=np.float64)
    raw_right_pose = np.asarray(raw_right_pose, dtype=np.float64)
    if not (np.isfinite(raw_left_pose).all() and np.isfinite(raw_right_pose).all()):
        return False
    return not (
        np.isclose(np.linalg.det(raw_left_pose), 0.0, atol=1e-6)
        or np.isclose(np.linalg.det(raw_right_pose), 0.0, atol=1e-6)
    )


def xr_tracking_ok(raw_left_pose, raw_right_pose, arm_pose_updated_at, now, tracking_timeout):
    """Return True when arm poses are valid and a fresh XR frame arrived within the timeout."""
    if arm_pose_updated_at <= 0.0:
        return False
    if not xr_arm_poses_valid(raw_left_pose, raw_right_pose):
        return False
    return (now - arm_pose_updated_at) <= tracking_timeout


def resolve_arm_ik_target(
    *,
    tracking_ok,
    left_wrist_pose,
    right_wrist_pose,
    current_lr_arm_q,
    current_lr_arm_dq,
    arm_ik,
    last_sol_q,
    tracking_fallback,
    frequency,
    home_ramp_rad_s=0.5,
):
    """Compute arm IK targets or apply hold/home safety fallback when tracking is lost."""
    if tracking_ok:
        sol_q, sol_tauff = arm_ik.solve_ik(
            left_wrist_pose,
            right_wrist_pose,
            current_lr_arm_q,
            current_lr_arm_dq,
        )
        return np.asarray(sol_q).copy(), np.asarray(sol_tauff).copy(), np.asarray(sol_q).copy()

    if last_sol_q is None:
        last_sol_q = np.asarray(current_lr_arm_q, dtype=np.float64).copy()
    else:
        last_sol_q = np.asarray(last_sol_q, dtype=np.float64).copy()

    if tracking_fallback == "home":
        max_step = home_ramp_rad_s / frequency
        sol_q = last_sol_q + np.clip(-last_sol_q, -max_step, max_step)
    else:
        sol_q = last_sol_q.copy()
    return sol_q, np.zeros_like(sol_q), sol_q
