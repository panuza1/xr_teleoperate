import numpy as np


BODY_JOINT_INDEX = {
    "hips": 0,
    "chest": 4,
    "left_shoulder": 7,
    "right_shoulder": 12,
}


class QuestUpperBodyRetargeter:
    """Retarget Quest torso tracking to the three G1 waist joints."""

    def __init__(self, smoothing=0.2, max_step=0.05):
        self.smoothing = float(smoothing)
        self.max_step = float(max_step)
        self.reference_frame = None
        self.output = np.zeros(3, dtype=np.float64)
        self.lower_limits = np.array([-0.8, -0.35, -0.35])
        self.upper_limits = np.array([0.8, 0.35, 0.35])

    @staticmethod
    def _normalize(vector):
        norm = np.linalg.norm(vector)
        if not np.isfinite(norm) or norm < 1e-6:
            raise ValueError("invalid torso segment")
        return vector / norm

    @staticmethod
    def _position(poses, name):
        position = poses[BODY_JOINT_INDEX[name], :3, 3]
        if not np.all(np.isfinite(position)):
            raise ValueError(f"invalid {name} position")
        return position

    def _torso_frame(self, poses):
        hips = self._position(poses, "hips")
        chest = self._position(poses, "chest")
        left_shoulder = self._position(poses, "left_shoulder")
        right_shoulder = self._position(poses, "right_shoulder")

        z_axis = self._normalize(chest - hips)
        y_axis = self._normalize(left_shoulder - right_shoulder)
        x_axis = self._normalize(np.cross(y_axis, z_axis))
        y_axis = self._normalize(np.cross(z_axis, x_axis))
        return np.column_stack((x_axis, y_axis, z_axis))

    @staticmethod
    def _zxy_angles(rotation):
        roll = np.arcsin(np.clip(rotation[2, 1], -1.0, 1.0))
        yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], rotation[2, 2])
        return np.array([yaw, roll, pitch])

    def calibrate(self, body_poses):
        poses = np.asarray(body_poses, dtype=np.float64)
        if poses.shape != (33, 4, 4):
            raise ValueError(f"expected body poses (33,4,4), got {poses.shape}")
        self.reference_frame = self._torso_frame(poses)

    @property
    def calibrated(self):
        return self.reference_frame is not None

    def retarget(self, body_poses):
        poses = np.asarray(body_poses, dtype=np.float64)
        if poses.shape != (33, 4, 4):
            raise ValueError(f"expected body poses (33,4,4), got {poses.shape}")
        if not self.calibrated:
            self.calibrate(poses)

        torso_delta = self.reference_frame.T @ self._torso_frame(poses)
        target = np.clip(
            self._zxy_angles(torso_delta),
            self.lower_limits,
            self.upper_limits,
        )
        bounded = np.clip(target, self.output - self.max_step, self.output + self.max_step)
        self.output += self.smoothing * (bounded - self.output)
        return self.output.copy()

    def reset(self):
        self.reference_frame = None
