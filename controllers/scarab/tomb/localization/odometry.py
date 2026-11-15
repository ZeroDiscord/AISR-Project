"""Odometry and pose estimation for the Pioneer 3-DX differential-drive mobile robot.

Provides pose tracking using:
  * GroundTruthPose: uses GPS + IMU (Webots) or ground truth (sim2d).
  * WheelOdometryPose: integrates wheel encoder angles using a 2nd-order
    Runge-Kutta / arc kinematics model.
  * FusedPose: uses wheel encoder displacements for (x, y) translation and
    IMU yaw for heading to eliminate rotational dead-reckoning drift.
  * PoseManager: coordinates pose sources and tracks position/heading drift
    against ground truth for experiment metrics.
"""

import math
from typing import Optional, Tuple

import numpy as np

from .. import config


def wrap_angle(angle: float) -> float:
    """Normalize an angle to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DifferentialDriveOdometry:
    """Dead reckoning from wheel encoders using differential-drive forward kinematics.

    Kinematics:
        s_L = R * phi_L,  s_R = R * phi_R
        delta_s = (delta_s_R + delta_s_L) / 2.0
        delta_theta = (delta_s_R - delta_s_L) / L

    Integration uses the 2nd-order Runge-Kutta (midpoint) method:
        theta_mid = theta + delta_theta / 2.0
        x_new = x + delta_s * cos(theta_mid)
        y_new = y + delta_s * sin(theta_mid)
        theta_new = wrap_angle(theta + delta_theta)
    """

    def __init__(
        self,
        wheel_radius: float = config.WHEEL_RADIUS,
        axle_length: float = config.AXLE_LENGTH,
        x0: float = 0.0,
        y0: float = 0.0,
        yaw0: float = 0.0,
    ):
        self.r = float(wheel_radius)
        self.L = float(axle_length)
        self.x = float(x0)
        self.y = float(y0)
        self.yaw = wrap_angle(float(yaw0))
        self.last_encoders: Optional[Tuple[float, float]] = None
        self.total_distance = 0.0

    def reset(self, x: float, y: float, yaw: float, encoders: Optional[Tuple[float, float]] = None):
        """Reset the odometric state to a known pose and reset encoder baselines."""
        self.x = float(x)
        self.y = float(y)
        self.yaw = wrap_angle(float(yaw))
        self.last_encoders = tuple(encoders) if encoders is not None else None
        self.total_distance = 0.0

    def update(self, encoders: Tuple[float, float]) -> Tuple[float, float, float]:
        """Integrate one encoder step.

        Parameters
        ----------
        encoders : tuple of (left_rad, right_rad)
            Cumulative wheel rotation angles in radians.

        Returns
        -------
        tuple of (x, y, yaw)
            Updated robot pose in world coordinates.
        """
        left, right = float(encoders[0]), float(encoders[1])
        if self.last_encoders is None:
            self.last_encoders = (left, right)
            return self.x, self.y, self.yaw

        d_left = left - self.last_encoders[0]
        d_right = right - self.last_encoders[1]
        self.last_encoders = (left, right)

        ds_l = self.r * d_left
        ds_r = self.r * d_right
        ds = (ds_r + ds_l) * 0.5
        d_theta = (ds_r - ds_l) / self.L

        mid_theta = self.yaw + d_theta * 0.5
        self.x += ds * math.cos(mid_theta)
        self.y += ds * math.sin(mid_theta)
        self.yaw = wrap_angle(self.yaw + d_theta)
        self.total_distance += abs(ds)

        return self.x, self.y, self.yaw

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw


class PoseSource:
    """Base interface for robot pose estimators."""

    def initialize(self, initial_pose: Tuple[float, float, float], initial_encoders: Tuple[float, float]):
        raise NotImplementedError

    def update(
        self,
        gt_pose: Tuple[float, float, float],
        encoders: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        raise NotImplementedError

    def reset(self, pose: Tuple[float, float, float], encoders: Optional[Tuple[float, float]] = None):
        raise NotImplementedError


class GroundTruthPose(PoseSource):
    """Passes through the simulator's true pose (GPS + IMU in Webots)."""

    def __init__(self):
        self._pose = (0.0, 0.0, 0.0)

    def initialize(self, initial_pose: Tuple[float, float, float], initial_encoders: Tuple[float, float]):
        self._pose = tuple(initial_pose)

    def update(
        self,
        gt_pose: Tuple[float, float, float],
        encoders: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        self._pose = tuple(gt_pose)
        return self._pose

    def reset(self, pose: Tuple[float, float, float], encoders: Optional[Tuple[float, float]] = None):
        self._pose = tuple(pose)


class WheelOdometryPose(PoseSource):
    """Pose estimation using pure dead-reckoning from wheel encoders."""

    def __init__(self, wheel_radius: float = config.WHEEL_RADIUS, axle_length: float = config.AXLE_LENGTH):
        self.odom = DifferentialDriveOdometry(wheel_radius, axle_length)

    def initialize(self, initial_pose: Tuple[float, float, float], initial_encoders: Tuple[float, float]):
        self.odom.reset(initial_pose[0], initial_pose[1], initial_pose[2], initial_encoders)

    def update(
        self,
        gt_pose: Tuple[float, float, float],
        encoders: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        return self.odom.update(encoders)

    def reset(self, pose: Tuple[float, float, float], encoders: Optional[Tuple[float, float]] = None):
        self.odom.reset(pose[0], pose[1], pose[2], encoders)


class FusedPose(PoseSource):
    """Sensor fusion: wheel encoder translation + IMU heading."""

    def __init__(self, wheel_radius: float = config.WHEEL_RADIUS):
        self.r = float(wheel_radius)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_encoders: Optional[Tuple[float, float]] = None

    def initialize(self, initial_pose: Tuple[float, float, float], initial_encoders: Tuple[float, float]):
        self.x, self.y, self.yaw = float(initial_pose[0]), float(initial_pose[1]), float(initial_pose[2])
        self.last_encoders = tuple(initial_encoders)

    def update(
        self,
        gt_pose: Tuple[float, float, float],
        encoders: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        # Heading comes directly from IMU yaw
        current_yaw = float(gt_pose[2])
        left, right = float(encoders[0]), float(encoders[1])

        if self.last_encoders is None:
            self.last_encoders = (left, right)
            self.yaw = current_yaw
            return self.x, self.y, self.yaw

        d_left = left - self.last_encoders[0]
        d_right = right - self.last_encoders[1]
        self.last_encoders = (left, right)

        ds = (d_right + d_left) * 0.5 * self.r
        mid_yaw = wrap_angle(self.yaw + wrap_angle(current_yaw - self.yaw) * 0.5)
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw = current_yaw

        return self.x, self.y, self.yaw

    def reset(self, pose: Tuple[float, float, float], encoders: Optional[Tuple[float, float]] = None):
        self.x, self.y, self.yaw = float(pose[0]), float(pose[1]), float(pose[2])
        self.last_encoders = tuple(encoders) if encoders is not None else None


class PoseManager:
    """Manages the robot's active pose source and tracks drift metrics against ground truth."""

    SOURCES = {
        "gt": GroundTruthPose,
        "odom": WheelOdometryPose,
        "fused": FusedPose,
    }

    def __init__(self, mode: str = "gt"):
        if mode not in self.SOURCES:
            raise ValueError(f"Unknown pose source '{mode}'. Expected one of {list(self.SOURCES.keys())}")
        self.mode = mode
        self.source = self.SOURCES[mode]()
        self.initialized = False
        self.drift_history = []  # list of (time, pos_error, yaw_error)

    def initialize(self, pose: Tuple[float, float, float], encoders: Tuple[float, float]):
        self.source.initialize(pose, encoders)
        self.initialized = True
        self.drift_history.clear()

    def update(
        self,
        sim_time: float,
        gt_pose: Tuple[float, float, float],
        encoders: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        """Update pose estimate and log drift against ground truth.

        Returns
        -------
        estimated_pose : (x, y, yaw)
        """
        if not self.initialized:
            self.initialize(gt_pose, encoders)

        est_pose = self.source.update(gt_pose, encoders)

        # Drift calculation against ground truth
        pos_err = math.hypot(est_pose[0] - gt_pose[0], est_pose[1] - gt_pose[1])
        yaw_err = abs(wrap_angle(est_pose[2] - gt_pose[2]))
        self.drift_history.append((sim_time, pos_err, yaw_err))

        return est_pose

    def reset(self, pose: Tuple[float, float, float], encoders: Optional[Tuple[float, float]] = None):
        self.source.reset(pose, encoders)
        self.drift_history.clear()

    def drift_summary(self) -> dict:
        """Return cumulative drift metrics."""
        if not self.drift_history:
            return {"max_pos_err": 0.0, "mean_pos_err": 0.0, "final_pos_err": 0.0, "max_yaw_err": 0.0}
        arr = np.asarray(self.drift_history)
        return {
            "max_pos_err": float(arr[:, 1].max()),
            "mean_pos_err": float(arr[:, 1].mean()),
            "final_pos_err": float(arr[-1, 1]),
            "max_yaw_err": float(arr[:, 2].max()),
            "mean_yaw_err": float(arr[:, 2].mean()),
            "final_yaw_err": float(arr[-1, 2]),
        }
