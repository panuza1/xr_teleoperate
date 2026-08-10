import unittest

import numpy as np

from teleop.robot_control.body_retargeting import QuestUpperBodyRetargeter
from teleop.utils.waist_safety import (
    WAIST_LIMITS_RAD,
    WAIST_MAX_DELTA_PER_CYCLE_RAD,
    body_tracking_ok,
    clamp_waist_target,
    rate_limit_sdk_weight,
    rate_limit_waist_target,
    resolve_waist_fallback,
)


class WaistSafetyTests(unittest.TestCase):
    def test_clamp_uses_conservative_limits(self):
        target = np.deg2rad([100.0, -30.0, 30.0])
        np.testing.assert_allclose(
            clamp_waist_target(target),
            np.array([WAIST_LIMITS_RAD[0], -WAIST_LIMITS_RAD[1], WAIST_LIMITS_RAD[2]]),
        )

    def test_rejects_non_finite_target(self):
        with self.assertRaises(ValueError):
            clamp_waist_target([0.0, np.nan, 0.0])

    def test_rate_limit_applies_per_axis_per_cycle(self):
        previous = np.zeros(3)
        target = np.ones(3)
        np.testing.assert_allclose(
            rate_limit_waist_target(target, previous),
            WAIST_MAX_DELTA_PER_CYCLE_RAD,
        )

    def test_body_tracking_requires_a_fresh_frame(self):
        self.assertFalse(body_tracking_ok(False, 9.9, 10.0, 0.5))
        self.assertFalse(body_tracking_ok(True, 9.0, 10.0, 0.5))
        self.assertFalse(body_tracking_ok(True, np.nan, 10.0, 0.5))
        self.assertTrue(body_tracking_ok(True, 9.9, 10.0, 0.5))

    def test_sdk_weight_ramps_in_both_directions(self):
        self.assertAlmostEqual(rate_limit_sdk_weight(0.0, 1.0, 0.002), 0.002)
        self.assertAlmostEqual(rate_limit_sdk_weight(1.0, 0.0, 0.002), 0.998)
        self.assertAlmostEqual(rate_limit_sdk_weight(0.999, 1.0, 0.002), 1.0)

    def test_hold_fallback_keeps_last_bounded_target(self):
        target = np.array([0.2, -0.1, 0.1])
        np.testing.assert_allclose(resolve_waist_fallback(target, "hold", 30.0), target)

    def test_home_fallback_moves_toward_zero(self):
        target = np.array([0.2, -0.1, 0.002])
        result = resolve_waist_fallback(
            target,
            "home",
            frequency=10.0,
            home_ramp_rad_s=0.05,
        )
        np.testing.assert_allclose(result, [0.195, -0.095, 0.0])

    def test_retargeter_smoothing_state_tracks_fallback_and_reset(self):
        retargeter = QuestUpperBodyRetargeter()
        retargeter.reference_frame = np.eye(3)
        retargeter.set_output([1.0, -1.0, 1.0])
        np.testing.assert_allclose(
            retargeter.output,
            [WAIST_LIMITS_RAD[0], -WAIST_LIMITS_RAD[1], WAIST_LIMITS_RAD[2]],
        )
        retargeter.reset()
        self.assertFalse(retargeter.calibrated)
        np.testing.assert_allclose(retargeter.output, np.zeros(3))


if __name__ == "__main__":
    unittest.main()
