import unittest

import numpy as np

from teleop.utils.xr_tracking_fallback import (
    resolve_arm_ik_target,
    xr_arm_poses_valid,
    xr_tracking_ok,
)


def _valid_pose():
    return np.eye(4, dtype=np.float64)


class DummyArmIK:
    def solve_ik(self, left_wrist_pose, right_wrist_pose, current_lr_arm_q, current_lr_arm_dq):
        return np.ones_like(current_lr_arm_q), np.zeros_like(current_lr_arm_q)


class XRTrackingFallbackTests(unittest.TestCase):
    def test_nan_pose_is_invalid(self):
        left = _valid_pose()
        left[0, 0] = np.nan
        self.assertFalse(xr_arm_poses_valid(left, _valid_pose()))

    def test_singular_pose_is_invalid(self):
        left = np.zeros((4, 4), dtype=np.float64)
        self.assertFalse(xr_arm_poses_valid(left, _valid_pose()))

    def test_stale_pose_is_tracking_lost(self):
        now = 10.0
        self.assertFalse(
            xr_tracking_ok(_valid_pose(), _valid_pose(), arm_pose_updated_at=8.0, now=now, tracking_timeout=0.5)
        )

    def test_fresh_pose_is_tracking_ok(self):
        now = 10.0
        self.assertTrue(
            xr_tracking_ok(_valid_pose(), _valid_pose(), arm_pose_updated_at=9.8, now=now, tracking_timeout=0.5)
        )

    def test_hold_fallback_keeps_last_solution(self):
        last_sol_q = np.array([0.2, -0.1, 0.3], dtype=np.float64)
        sol_q, sol_tauff, updated_last = resolve_arm_ik_target(
            tracking_ok=False,
            left_wrist_pose=_valid_pose(),
            right_wrist_pose=_valid_pose(),
            current_lr_arm_q=np.zeros(3),
            current_lr_arm_dq=np.zeros(3),
            arm_ik=DummyArmIK(),
            last_sol_q=last_sol_q,
            tracking_fallback="hold",
            frequency=30.0,
        )
        np.testing.assert_allclose(sol_q, last_sol_q)
        np.testing.assert_allclose(updated_last, last_sol_q)
        np.testing.assert_allclose(sol_tauff, np.zeros(3))

    def test_home_fallback_moves_toward_zero_with_bounded_step(self):
        last_sol_q = np.array([1.0, -1.0, 0.5], dtype=np.float64)
        sol_q, _, updated_last = resolve_arm_ik_target(
            tracking_ok=False,
            left_wrist_pose=_valid_pose(),
            right_wrist_pose=_valid_pose(),
            current_lr_arm_q=np.zeros(3),
            current_lr_arm_dq=np.zeros(3),
            arm_ik=DummyArmIK(),
            last_sol_q=last_sol_q,
            tracking_fallback="home",
            frequency=10.0,
            home_ramp_rad_s=0.5,
        )
        np.testing.assert_allclose(sol_q, last_sol_q + np.array([-0.05, 0.05, -0.05]))
        np.testing.assert_allclose(updated_last, sol_q)


if __name__ == "__main__":
    unittest.main()
