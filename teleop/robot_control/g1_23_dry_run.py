#!/usr/bin/env python3
"""Offline synthetic Quest-controller → G1_23 IK preview; no DDS is initialized."""

import argparse
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
from teleop.robot_control.robot_arm import G1_23_ARM_POSITION_LIMITS, G1_23_ArmController
from teleop.robot_control.robot_arm_ik import G1_23_ArmIK
sys.path.insert(0, str(REPO_ROOT / "teleop" / "televuer" / "src"))
from televuer.tv_wrapper import T_OPENXR_ROBOT, T_ROBOT_OPENXR, TeleVuerWrapper

ARM_SLOTS = {"left": (15, 16, 17, 18, 19), "right": (22, 23, 24, 25, 26)}


def controller_pose_for_target(target):
    unitree_pose = target.copy()
    unitree_pose[:3, 3] -= [0.15, 0.0, 0.45]
    return T_OPENXR_ROBOT @ unitree_pose @ T_ROBOT_OPENXR


def build_preview(scenario="reachable"):
    if scenario not in ("reachable", "unreachable-left"):
        raise ValueError(f"unsupported scenario: {scenario}")
    ik = G1_23_ArmIK()
    measured_q = np.zeros(10)
    data = ik.reduced_robot.model.createData()
    import pinocchio as pin
    pin.framesForwardKinematics(ik.reduced_robot.model, data, measured_q)
    left_target = np.array(data.oMf[ik.L_hand_id].homogeneous)
    right_target = np.array(data.oMf[ik.R_hand_id].homogeneous)
    if scenario == "unreachable-left":
        left_target[:3, 3] += [2.0, 0.0, 0.0]

    now = time.monotonic()
    tvuer = SimpleNamespace(
        head_pose=np.eye(4),
        left_arm_pose=controller_pose_for_target(left_target),
        right_arm_pose=controller_pose_for_target(right_target),
        motion_data_ready=True,
        hand_data_updated_at=now,
    )
    wrapper = TeleVuerWrapper.__new__(TeleVuerWrapper)
    wrapper.use_hand_tracking = False
    wrapper.use_body_tracking = False
    wrapper.use_controller_input = False
    wrapper.arm_reference_mode = "head_yaw"
    wrapper.tvuer = tvuer
    tele_data = wrapper.get_tele_data()
    controller_poses = (tvuer.left_arm_pose, tvuer.right_arm_pose)
    pose_valid = [
        pose.shape == (4, 4) and np.isfinite(pose).all()
        and np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6)
        for pose in controller_poses
    ]

    q, tau = ik.solve_ik(
        tele_data.left_wrist_pose, tele_data.right_wrist_pose,
        measured_q, np.zeros(10),
    )
    arm = G1_23_ArmController.__new__(G1_23_ArmController)
    arm.control_dt = 1.0 / 250.0
    arm.get_current_dual_arm_q = lambda: measured_q
    rate_limited_q = arm.clip_arm_q_target(q, 20.0)
    finite_limited = (
        np.isfinite(q).all() and np.isfinite(tau).all()
        and np.all(q >= G1_23_ARM_POSITION_LIMITS[:, 0])
        and np.all(q <= G1_23_ARM_POSITION_LIMITS[:, 1])
    )
    rate_limit_respected = np.max(np.abs(rate_limited_q - measured_q)) <= 20.0 * arm.control_dt + 1e-12
    status = ik.last_ik_status
    return {
        "scenario": scenario,
        "controller_poses_valid": all(pose_valid),
        "left_controller_pose_valid": pose_valid[0],
        "right_controller_pose_valid": pose_valid[1],
        "coordinate_transform_left_m": tele_data.left_wrist_pose,
        "coordinate_transform_right_m": tele_data.right_wrist_pose,
        "tracking_fresh": 0.0 <= time.monotonic() - now < 0.5,
        "ik": status,
        "target_q": q,
        "tau": tau,
        "rate_limited_q": rate_limited_q,
        "joint_limits_valid": finite_limited,
        "rate_limit_respected": rate_limit_respected,
        "would_be_command_permitted": bool(
            all(pose_valid) and finite_limited and rate_limit_respected
            and 0.0 <= time.monotonic() - now < 0.5 and status.get("accepted")
            and (status.get("left_accepted") or status.get("right_accepted"))
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("reachable", "unreachable-left"), default="reachable")
    args = parser.parse_args()
    result = build_preview(args.scenario)
    status = result["ik"]
    print(f"OFFLINE ONLY — scenario={result['scenario']} — DDS publishers: none")
    print(
        f"left_controller_pose_valid={result['left_controller_pose_valid']} "
        f"right_controller_pose_valid={result['right_controller_pose_valid']} "
        f"synthetic_tracking_fresh={result['tracking_fresh']}"
    )
    print(
        f"IK={status.get('reason')} left_residual={status.get('left_residual_m')}m "
        f"right_residual={status.get('right_residual_m')}m "
        f"left_accepted={status.get('left_accepted')} right_accepted={status.get('right_accepted')}"
    )
    print(f"coordinate_transform_left=\n{result['coordinate_transform_left_m']}")
    print(f"coordinate_transform_right=\n{result['coordinate_transform_right_m']}")
    print(f"target_q_rad={np.round(result['target_q'], 4)}")
    for side, slots, values in (
        ("left", ARM_SLOTS["left"], result["target_q"][:5]),
        ("right", ARM_SLOTS["right"], result["target_q"][5:]),
    ):
        print(f"would_be_{side}_joint_targets=" + " ".join(f"{slot}:{value:+.4f}" for slot, value in zip(slots, values)))
    print(f"joint_limits_valid={result['joint_limits_valid']} rate_limit_respected={result['rate_limit_respected']} rate_limited_q_rad={np.round(result['rate_limited_q'], 4)}")
    print(f"would_be_command_permitted={result['would_be_command_permitted']} (preview only; never published)")
    return 0 if result["would_be_command_permitted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
