import os
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin

from teleop.robot_control.robot_arm_ik import G1_29_ArmIK


ROOT = Path(__file__).parents[2]


class G129ReachScalingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.previous_cwd = Path.cwd()
        os.chdir(ROOT / "teleop")
        cls.ik = G1_29_ArmIK()
        cls.scaled_ik = G1_29_ArmIK(reach_gain=1.2)

    @classmethod
    def tearDownClass(cls):
        os.chdir(cls.previous_cwd)

    @staticmethod
    def wrist_targets():
        left = np.eye(4)
        right = np.eye(4)
        left[:3, :3] = np.diag([-1.0, 1.0, -1.0])
        right[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        left[:3, 3] = [0.25, 0.25, 0.10]
        right[:3, 3] = [0.25, -0.25, 0.10]
        return left, right

    def test_default_gain_preserves_wrist_poses_exactly(self):
        left, right = self.wrist_targets()

        scaled_left, scaled_right = self.ik.scale_arms(left, right)

        np.testing.assert_array_equal(scaled_left, left)
        np.testing.assert_array_equal(scaled_right, right)

    def test_gain_scales_translation_from_each_shoulder_only(self):
        left, right = self.wrist_targets()

        scaled_left, scaled_right = self.scaled_ik.scale_arms(left, right)

        np.testing.assert_allclose(
            scaled_left[:3, 3], [0.30000144, 0.279956, 0.061644]
        )
        np.testing.assert_allclose(
            scaled_right[:3, 3], [0.30000144, -0.279958, 0.061644]
        )
        np.testing.assert_array_equal(scaled_left[:3, :3], left[:3, :3])
        np.testing.assert_array_equal(scaled_right[:3, :3], right[:3, :3])
        np.testing.assert_array_equal(left[:3, 3], [0.25, 0.25, 0.10])
        np.testing.assert_array_equal(right[:3, 3], [0.25, -0.25, 0.10])

    def test_cli_exposes_arm_reach_gain(self):
        result = subprocess.run(
            [sys.executable, "teleop_hand_and_arm.py", "--help"],
            cwd=ROOT / "teleop",
            capture_output=True,
            text=True,
            check=True,
        )

        self.assertIn("--arm-reach-gain", result.stdout)

    def test_ik_converges_within_joint_limits(self):
        left, right = self.wrist_targets()
        left[:3, :3] = np.eye(3)
        right[:3, :3] = np.eye(3)
        reaches = []
        elbows = []

        for ik in (self.ik, self.scaled_ik):
            q, _ = ik.solve_ik(left, right, np.zeros(14), np.zeros(14))
            self.assertTrue(ik.opti.stats()["success"])
            self.assertTrue(np.all(np.isfinite(q)))
            self.assertTrue(np.all(q >= ik.reduced_robot.model.lowerPositionLimit))
            self.assertTrue(np.all(q <= ik.reduced_robot.model.upperPositionLimit))

            data = ik.reduced_robot.model.createData()
            pin.framesForwardKinematics(ik.reduced_robot.model, data, q)
            reaches.append(
                np.linalg.norm(
                    data.oMf[ik.L_hand_id].translation - ik.left_shoulder_origin
                )
            )
            elbows.append(q[3])

        self.assertGreater(reaches[1], reaches[0])
        self.assertGreater(elbows[1], elbows[0])

    def test_invalid_gain_is_rejected_before_model_loading(self):
        for gain in (0.0, -1.0, np.inf, np.nan):
            with self.subTest(gain=gain), self.assertRaises(ValueError):
                G1_29_ArmIK(reach_gain=gain)

    def test_offline_mujoco_validation_compares_multiple_gains(self):
        from teleop.robot_control.validate_g1_29_reach_scaling import (
            validate_reach_scaling,
        )

        results = validate_reach_scaling((1.0, 1.1, 1.2))

        self.assertEqual([row["gain"] for row in results], [1.0, 1.1, 1.2])
        self.assertTrue(all(row["converged"] for row in results))
        self.assertTrue(all(row["within_limits"] for row in results))
        self.assertGreater(results[-1]["left_reach"], results[0]["left_reach"])
        self.assertGreater(results[-1]["left_elbow"], results[0]["left_elbow"])

    def test_solve_applies_gain_once(self):
        left, right = self.wrist_targets()
        left[:3, :3] = np.eye(3)
        right[:3, :3] = np.eye(3)
        scaled_ik = G1_29_ArmIK(reach_gain=1.2)
        reference_ik = G1_29_ArmIK()
        scaled_left, scaled_right = scaled_ik.scale_arms(left, right)

        scaled_q, _ = scaled_ik.solve_ik(
            left, right, np.zeros(14), np.zeros(14)
        )
        reference_q, _ = reference_ik.solve_ik(
            scaled_left, scaled_right, np.zeros(14), np.zeros(14)
        )

        np.testing.assert_allclose(scaled_q, reference_q, atol=1e-8)


if __name__ == "__main__":
    unittest.main()
