import unittest
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).parents[1] / "televuer" / "src"))

from televuer.tv_wrapper import T_OPENXR_ROBOT, T_ROBOT_OPENXR, TeleVuerWrapper
from teleop.robot_control.robot_arm import (
    G1_23_ARM_POSITION_LIMITS,
    G1_23_ArmController,
    G1_23_JointArmIndex,
)
from teleop.robot_control.robot_arm_ik import G1_23_ArmIK
from teleop.utils.weighted_moving_filter import WeightedMovingFilter
from teleop.utils.xr_tracking_fallback import resolve_arm_ik_target


class G123ControllerIKTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ik = G1_23_ArmIK()

    def setUp(self):
        self.ik.init_data = np.zeros(self.ik.reduced_robot.model.nq)
        self.reset_filter()

    def reset_filter(self):
        self.ik.smooth_filter = WeightedMovingFilter(
            np.array([0.4, 0.3, 0.2, 0.1]), self.ik.reduced_robot.model.nq
        )

    def fk_targets(self, q):
        data = self.ik.reduced_robot.model.createData()
        pin.framesForwardKinematics(self.ik.reduced_robot.model, data, q)
        return (
            np.array(data.oMf[self.ik.L_hand_id].homogeneous),
            np.array(data.oMf[self.ik.R_hand_id].homogeneous),
        )

    @staticmethod
    def controller_pose_for_target(target):
        world_arm = target.copy()
        world_arm[:3, 3] -= [0.15, 0.0, 0.45]
        return T_OPENXR_ROBOT @ world_arm @ T_ROBOT_OPENXR

    def teleop_targets(self, left_target, right_target):
        tvuer = SimpleNamespace(
            head_pose=np.eye(4),
            left_arm_pose=self.controller_pose_for_target(left_target),
            right_arm_pose=self.controller_pose_for_target(right_target),
            motion_data_ready=True,
            hand_data_updated_at=0.0,
        )
        wrapper = TeleVuerWrapper.__new__(TeleVuerWrapper)
        wrapper.use_hand_tracking = False
        wrapper.use_body_tracking = False
        wrapper.use_controller_input = False
        wrapper.arm_reference_mode = "head_yaw"
        wrapper.tvuer = tvuer
        return wrapper.get_tele_data()

    def solve(self, data, current_q):
        return self.ik.solve_ik(
            data.left_wrist_pose,
            data.right_wrist_pose,
            current_q,
            np.zeros(10),
        )

    def assert_solution(self, q, tau):
        self.assertEqual(q.shape, (10,))
        self.assertEqual(tau.shape, (10,))
        self.assertTrue(np.isfinite(q).all())
        self.assertTrue(np.isfinite(tau).all())
        np.testing.assert_array_less(
            self.ik.reduced_robot.model.lowerPositionLimit - 1e-8, q
        )
        np.testing.assert_array_less(
            q, self.ik.reduced_robot.model.upperPositionLimit + 1e-8
        )
        np.testing.assert_array_less(
            G1_23_ARM_POSITION_LIMITS[:, 0] - 1e-8, q
        )
        np.testing.assert_array_less(
            q, G1_23_ARM_POSITION_LIMITS[:, 1] + 1e-8
        )

    def test_synthetic_controller_targets_solve_independently_and_together(self):
        zero = np.zeros(10)
        left_move = zero.copy()
        left_move[:5] = [0.04, 0.025, 0.03, 0.04, 0.02]
        right_move = zero.copy()
        right_move[5:] = [-0.035, 0.02, -0.025, 0.035, -0.02]
        both_move = left_move.copy()
        both_move[1] += 0.02
        both_move[5:] = right_move[5:]

        base_data = self.teleop_targets(*self.fk_targets(zero))
        base_q, base_tau = self.solve(base_data, zero)
        self.assertTrue(self.ik.opti.stats()["success"])
        self.assert_solution(base_q, base_tau)

        left_data = self.teleop_targets(*self.fk_targets(left_move))
        self.reset_filter()
        left_q, left_tau = self.solve(left_data, zero)
        self.assertTrue(self.ik.opti.stats()["success"])
        self.assert_solution(left_q, left_tau)
        self.assertGreater(np.linalg.norm(left_q[:5] - base_q[:5]), 0.04)
        self.assertLess(np.max(np.abs(left_q[5:] - base_q[5:])), 0.01)

        right_data = self.teleop_targets(*self.fk_targets(right_move))
        self.reset_filter()
        right_q, right_tau = self.solve(right_data, zero)
        self.assertTrue(self.ik.opti.stats()["success"])
        self.assert_solution(right_q, right_tau)
        self.assertGreater(np.linalg.norm(right_q[5:] - left_q[5:]), 0.02)
        self.assertLess(np.max(np.abs(right_q[:5] - base_q[:5])), 0.01)

        both_data = self.teleop_targets(*self.fk_targets(both_move))
        self.reset_filter()
        both_q, both_tau = self.solve(both_data, zero)
        self.assertTrue(self.ik.opti.stats()["success"])
        self.assert_solution(both_q, both_tau)
        self.assertGreater(np.linalg.norm(both_q[:5] - base_q[:5]), 0.01)
        self.assertGreater(np.linalg.norm(both_q[5:] - base_q[5:]), 0.01)
        self.reset_filter()
        previous = base_q
        for fraction in (0.25, 0.5, 0.75, 1.0):
            step_target = left_move.copy()
            step_target[:5] *= fraction
            step_data = self.teleop_targets(*self.fk_targets(step_target))
            current, _ = self.solve(step_data, previous)
            self.assert_solution(current, np.zeros(10))
            self.assertLess(np.max(np.abs(current - previous)), 0.15)
            previous = current

    def test_joint_order_and_publisher_rate_limit_match_g1_23_arms(self):
        model = self.ik.reduced_robot.model
        expected = (
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
        )
        self.assertEqual(tuple(model.names[j] for j in range(1, model.njoints)), expected)
        self.assertEqual(tuple(joint.value for joint in G1_23_JointArmIndex),
                         (15, 16, 17, 18, 19, 22, 23, 24, 25, 26))

        controller = G1_23_ArmController.__new__(G1_23_ArmController)
        controller.control_dt = 1.0 / 250.0
        controller.arm_velocity_limit = 20.0
        controller.get_current_dual_arm_q = lambda: np.zeros(10)
        clipped = controller.clip_arm_q_target(np.full(10, 2.0), 20.0)
        self.assertLessEqual(np.max(np.abs(clipped)), 20.0 * controller.control_dt + 1e-12)

        measured_q = np.linspace(-0.1, 0.1, 10)
        held_q, _, _ = resolve_arm_ik_target(
            tracking_ok=False,
            left_wrist_pose=np.eye(4),
            right_wrist_pose=np.eye(4),
            current_lr_arm_q=measured_q,
            current_lr_arm_dq=np.zeros(10),
            arm_ik=self.ik,
            last_sol_q=None,
            tracking_fallback="hold",
            frequency=30.0,
        )
        np.testing.assert_allclose(held_q, measured_q)
        controller.get_current_dual_arm_q = lambda: held_q
        reconnect_step = controller.clip_arm_q_target(np.full(10, 1.0), 20.0)
        self.assertLessEqual(
            np.max(np.abs(reconnect_step - measured_q)),
            20.0 * controller.control_dt + 1e-12,
        )

    def test_unreachable_arm_is_held_and_reachable_arm_recovers(self):
        zero = np.zeros(10)
        left_target, right_target = self.fk_targets(zero)
        unreachable = left_target.copy()
        unreachable[:3, 3] += [10.0, 10.0, 10.0]
        data = self.teleop_targets(unreachable, right_target)
        q, tau = self.solve(data, zero)
        self.assert_solution(q, tau)
        np.testing.assert_allclose(q[:5], zero[:5])
        np.testing.assert_allclose(tau[:5], 0.0)
        self.assertFalse(self.ik.last_ik_status["left_accepted"])
        self.assertTrue(self.ik.last_ik_status["right_accepted"])
        self.assertGreater(self.ik.last_ik_status["left_residual_m"], 0.05)
        np.testing.assert_allclose(self.ik.init_data[:5], zero[:5])

        moved = np.array([0.08, 0.04, 0.05, 0.04, 0.02, 0, 0, 0, 0, 0])
        reachable = self.teleop_targets(*self.fk_targets(moved))
        q, _ = self.solve(reachable, zero)
        self.assertTrue(self.ik.last_ik_status["left_accepted"])
        self.assertGreater(np.linalg.norm(q[:5]), 0.05)

    def test_stale_left_controller_holds_left_arm_while_right_continues(self):
        measured = np.zeros(10)
        right_move = measured.copy()
        right_move[5:] = [-0.04, 0.03, -0.02, 0.04, -0.02]
        left_target, right_target = self.fk_targets(right_move)
        q, tau, _ = resolve_arm_ik_target(
            tracking_ok=False,
            left_wrist_pose=left_target,
            right_wrist_pose=right_target,
            current_lr_arm_q=measured,
            current_lr_arm_dq=np.zeros(10),
            arm_ik=self.ik,
            last_sol_q=None,
            tracking_fallback="hold",
            frequency=30.0,
            left_tracking_ok=False,
            right_tracking_ok=True,
        )
        np.testing.assert_allclose(q[:5], measured[:5])
        np.testing.assert_allclose(tau[:5], 0.0)
        self.assertFalse(self.ik.last_ik_status["left_accepted"])
        self.assertTrue(self.ik.last_ik_status["right_accepted"])
        self.assertGreater(np.linalg.norm(q[5:]), 0.02)

    def test_residual_threshold_accepts_boundary_and_holds_above_it(self):
        zero = np.zeros(10)
        left, right = self.fk_targets(zero)
        for offset, expected in ((0.05, True), (0.05001, False)):
            with self.subTest(offset=offset):
                left_target, right_target = left.copy(), right.copy()
                left_target[0, 3] += offset
                data = self.teleop_targets(left_target, right_target)
                self.reset_filter()
                with (
                    patch.object(self.ik.opti, "solve", return_value=None),
                    patch.object(self.ik.opti, "value", return_value=zero.copy()),
                    patch.object(self.ik.opti, "stats", return_value={"success": True}),
                ):
                    q, _ = self.solve(data, zero)
                self.assertEqual(self.ik.last_ik_status["left_accepted"], expected)
                if expected:
                    self.assertAlmostEqual(self.ik.last_ik_status["left_residual_m"], offset, places=6)
                else:
                    np.testing.assert_allclose(q[:5], zero[:5])

    def test_nonfinite_pose_and_joint_solutions_are_rejected(self):
        zero = np.zeros(10)
        left, right = self.fk_targets(zero)
        invalid = left.copy()
        invalid[0, 3] = np.nan
        q, tau = self.ik.solve_ik(invalid, right, zero, np.zeros(10))
        np.testing.assert_allclose(q[:5], zero[:5])
        np.testing.assert_allclose(tau[:5], 0.0)
        self.assertFalse(self.ik.last_ik_status["left_accepted"])
        self.assertTrue(self.ik.last_ik_status["right_accepted"])
        self.assertEqual(self.ik.last_ik_status["left_reason"], "invalid wrist pose")
        self.assertTrue(np.allclose(self.ik.smooth_filter._data_queue[-1][:5], zero[:5]))
        q, tau = self.ik.solve_ik(b"\x00", right, zero, np.zeros(10))
        np.testing.assert_allclose(q[:5], zero[:5])
        np.testing.assert_allclose(tau[:5], 0.0)
        self.assertFalse(self.ik.last_ik_status["left_accepted"])
        self.assertTrue(self.ik.last_ik_status["right_accepted"])

        data = self.teleop_targets(left, right)
        out_of_limits = zero.copy()
        out_of_limits[0] = self.ik.reduced_robot.model.upperPositionLimit[0] + 0.1
        with (
            patch.object(self.ik.opti, "solve", return_value=None),
            patch.object(self.ik.opti, "value", return_value=out_of_limits),
            patch.object(self.ik.opti, "stats", return_value={"success": True}),
        ):
            q, tau = self.solve(data, zero)
        np.testing.assert_allclose(q[:5], zero[:5])
        np.testing.assert_allclose(tau[:5], 0.0)
        self.assertFalse(self.ik.last_ik_status["left_accepted"])
        self.assertTrue(self.ik.last_ik_status["right_accepted"])

        nonfinite_solution = zero.copy()
        nonfinite_solution[0] = np.inf
        seed_before = self.ik.init_data.copy()
        filter_before = list(self.ik.smooth_filter._data_queue)
        with (
            patch.object(self.ik.opti, "solve", return_value=None),
            patch.object(self.ik.opti, "value", return_value=nonfinite_solution),
            patch.object(self.ik.opti, "stats", return_value={"success": True}),
        ):
            q, tau = self.solve(data, zero)
        np.testing.assert_allclose(q, zero)
        np.testing.assert_allclose(tau, 0.0)
        self.assertEqual(len(self.ik.smooth_filter._data_queue), len(filter_before))
        np.testing.assert_allclose(self.ik.init_data, seed_before)

    def test_solver_exception_does_not_change_seed_or_filter(self):
        data = self.teleop_targets(*self.fk_targets(np.zeros(10)))
        current = np.linspace(-0.1, 0.1, 10)
        seed_before = self.ik.init_data.copy()
        filter_before = list(self.ik.smooth_filter._data_queue)
        with patch.object(self.ik.opti, "solve", side_effect=RuntimeError("forced IPOPT failure")):
            held_q, held_tau = self.solve(data, current)
        np.testing.assert_allclose(held_q, current)
        np.testing.assert_allclose(held_tau, 0.0)
        np.testing.assert_allclose(self.ik.init_data, seed_before)
        self.assertEqual(self.ik.smooth_filter._data_queue, filter_before)

        with (
            patch.object(self.ik.opti, "solve", return_value=None),
            patch.object(self.ik.opti, "stats", return_value={"success": False}),
        ):
            held_q, held_tau = self.solve(data, current)
        np.testing.assert_allclose(held_q, current)
        np.testing.assert_allclose(held_tau, 0.0)
        np.testing.assert_allclose(self.ik.init_data, seed_before)
        self.assertEqual(self.ik.smooth_filter._data_queue, filter_before)

    def test_rotation_with_one_stationary_wrist_keeps_ik_solver_valid(self):
        zero = np.zeros(10)
        left_target, right_target = self.fk_targets(zero)
        angle = 0.8
        rotation_x = np.array([
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle), -np.sin(angle)],
            [0.0, np.sin(angle), np.cos(angle)],
        ])
        left_target[:3, :3] = rotation_x @ left_target[:3, :3]
        data = self.teleop_targets(left_target, right_target)
        q, tau = self.solve(data, zero)
        self.assertTrue(self.ik.opti.stats()["success"], self.ik.opti.stats())
        self.assert_solution(q, tau)
        self.assertGreater(np.linalg.norm(q[:5]), 0.1)
        self.assertLess(np.max(np.abs(q[5:])), 0.01)
        fk_data = self.ik.reduced_robot.model.createData()
        pin.framesForwardKinematics(self.ik.reduced_robot.model, fk_data, q)
        self.assertLess(np.linalg.norm(fk_data.oMf[self.ik.L_hand_id].translation - left_target[:3, 3]), 0.01)
        self.assertGreater(
            np.linalg.norm(pin.log3(fk_data.oMf[self.ik.L_hand_id].rotation @ left_target[:3, :3].T)),
            0.05,
        )
        self.assertTrue(self.ik.last_ik_status["left_accepted"])
        self.assertTrue(self.ik.last_ik_status["right_accepted"])


if __name__ == "__main__":
    unittest.main()
