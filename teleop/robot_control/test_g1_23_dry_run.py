import unittest

from teleop.robot_control.g1_23_dry_run import build_preview


class G123DryRunTest(unittest.TestCase):
    def test_reachable_synthetic_controller_pose_passes_offline_preview(self):
        result = build_preview("reachable")
        self.assertTrue(result["controller_poses_valid"])
        self.assertTrue(result["tracking_fresh"])
        self.assertTrue(result["joint_limits_valid"])
        self.assertTrue(result["rate_limit_respected"])
        self.assertTrue(result["would_be_command_permitted"])
        self.assertTrue(result["ik"]["left_accepted"])
        self.assertTrue(result["ik"]["right_accepted"])

    def test_unreachable_synthetic_left_pose_reports_residual_without_hard_cutoff(self):
        result = build_preview("unreachable-left")
        self.assertTrue(result["controller_poses_valid"])
        self.assertTrue(result["joint_limits_valid"])
        self.assertTrue(result["rate_limit_respected"])
        self.assertGreater(result["ik"]["left_residual_m"], 0.05)
        # Official-style best-effort IK can report a valid bounded solution even
        # when the desired wrist position is outside the physical workspace.
        # Solver failure is still permitted to hold the measured joints.
        self.assertEqual(
            result["would_be_command_permitted"],
            bool(result["ik"]["accepted"]),
        )


if __name__ == "__main__":
    unittest.main()
