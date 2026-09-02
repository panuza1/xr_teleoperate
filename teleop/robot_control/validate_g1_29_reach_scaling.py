import argparse
import os
from pathlib import Path

import mujoco
import numpy as np

from teleop.robot_control.robot_arm_ik import G1_29_ArmIK


ROOT = Path(__file__).parents[2]
ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


def _wrist_targets():
    left = np.eye(4)
    right = np.eye(4)
    left[:3, 3] = [0.25, 0.25, 0.10]
    right[:3, 3] = [0.25, -0.25, 0.10]
    return left, right


def _mujoco_reach(model, q):
    data = mujoco.MjData(model)
    for name, value in zip(ARM_JOINT_NAMES, q):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)

    reaches = []
    for side in ("left", "right"):
        shoulder_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_shoulder_pitch_joint"
        )
        wrist_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wrist_yaw_link"
        )
        wrist_rotation = data.xmat[wrist_id].reshape(3, 3)
        ee_position = data.xpos[wrist_id] + wrist_rotation @ np.array([0.05, 0, 0])
        reaches.append(float(np.linalg.norm(ee_position - data.xanchor[shoulder_id])))
    return reaches


def validate_reach_scaling(gains=(1.0, 1.05, 1.1, 1.15, 1.2)):
    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "assets/g1/g1_body29_hand14.xml")
    )
    left, right = _wrist_targets()
    results = []
    previous_cwd = Path.cwd()
    os.chdir(ROOT / "teleop")
    try:
        for gain in gains:
            ik = G1_29_ArmIK(reach_gain=gain)
            scaled_left, scaled_right = ik.scale_arms(left, right)
            q, _ = ik.solve_ik(left, right, np.zeros(14), np.zeros(14))
            lower = ik.reduced_robot.model.lowerPositionLimit
            upper = ik.reduced_robot.model.upperPositionLimit
            left_reach, right_reach = _mujoco_reach(model, q)
            results.append(
                {
                    "gain": float(gain),
                    "converged": bool(ik.opti.stats().get("success")),
                    "within_limits": bool(np.all(q >= lower) and np.all(q <= upper)),
                    "left_target_reach": float(
                        np.linalg.norm(scaled_left[:3, 3] - ik.left_shoulder_origin)
                    ),
                    "right_target_reach": float(
                        np.linalg.norm(scaled_right[:3, 3] - ik.right_shoulder_origin)
                    ),
                    "left_reach": left_reach,
                    "right_reach": right_reach,
                    "left_elbow": float(q[3]),
                    "right_elbow": float(q[10]),
                }
            )
    finally:
        os.chdir(previous_cwd)
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Offline G1_29 shoulder-relative reach validation; sends no DDS commands."
    )
    parser.add_argument(
        "--gains", nargs="+", type=float, default=[1.0, 1.05, 1.1, 1.15, 1.2]
    )
    args = parser.parse_args()

    print("gain converged limits target_L reach_L elbow_L target_R reach_R elbow_R")
    for row in validate_reach_scaling(args.gains):
        print(
            f"{row['gain']:.3f} {str(row['converged']):>9} "
            f"{str(row['within_limits']):>6} {row['left_target_reach']:.4f} "
            f"{row['left_reach']:.4f} {row['left_elbow']:.4f} "
            f"{row['right_target_reach']:.4f} {row['right_reach']:.4f} "
            f"{row['right_elbow']:.4f}"
        )


if __name__ == "__main__":
    main()
