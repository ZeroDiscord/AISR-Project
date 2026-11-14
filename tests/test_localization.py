"""Unit tests for the differential-drive odometry and pose estimation package."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controllers", "scarab"))

from tomb import config
from tomb.localization import (
    DifferentialDriveOdometry,
    FusedPose,
    GroundTruthPose,
    PoseManager,
    WheelOdometryPose,
    wrap_angle,
)


def test_angle_wrapping():
    assert math.isclose(wrap_angle(0.0), 0.0)
    assert math.isclose(wrap_angle(math.pi), -math.pi)
    assert math.isclose(wrap_angle(-math.pi), -math.pi)
    assert math.isclose(wrap_angle(3 * math.pi), -math.pi)
    assert math.isclose(wrap_angle(-3 * math.pi), -math.pi)
    assert math.isclose(wrap_angle(math.pi / 2), math.pi / 2)
    assert math.isclose(wrap_angle(-math.pi / 2), -math.pi / 2)


def test_odometry_straight_line():
    odom = DifferentialDriveOdometry(wheel_radius=0.1, axle_length=0.4, x0=1.0, y0=2.0, yaw0=0.0)
    # First call sets baseline
    odom.update((0.0, 0.0))
    # Move both wheels forward by 10 rad -> 1.0 m forward along x
    x, y, yaw = odom.update((10.0, 10.0))
    assert math.isclose(x, 2.0, abs_tol=1e-6)
    assert math.isclose(y, 2.0, abs_tol=1e-6)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6)
    assert math.isclose(odom.total_distance, 1.0, abs_tol=1e-6)


def test_odometry_turn_in_place():
    r = 0.1
    L = 0.4
    odom = DifferentialDriveOdometry(wheel_radius=r, axle_length=L, x0=0.0, y0=0.0, yaw0=0.0)
    odom.update((0.0, 0.0))
    # Rotate right wheel forward by dphi, left wheel backward by dphi
    # delta_theta = (r * dphi - (-r * dphi)) / L = 2 * r * dphi / L
    # For 90 deg (pi/2) turn: dphi = (pi/2) * L / (2 * r) = (pi/2) * 0.4 / 0.2 = pi
    dphi = (math.pi / 2.0) * L / (2.0 * r)
    x, y, yaw = odom.update((-dphi, dphi))
    assert math.isclose(x, 0.0, abs_tol=1e-6)
    assert math.isclose(y, 0.0, abs_tol=1e-6)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-6)


def test_odometry_arc():
    r = 0.1
    L = 0.4
    odom = DifferentialDriveOdometry(wheel_radius=r, axle_length=L, x0=0.0, y0=0.0, yaw0=0.0)
    odom.update((0.0, 0.0))
    # Drive a quarter circle of radius R_c = 1.0 m turning left
    # Arc length s = (pi / 2) * R_c
    # Inner wheel (left) radius: R_c - L/2 = 0.8 m -> s_L = (pi / 2) * 0.8
    # Outer wheel (right) radius: R_c + L/2 = 1.2 m -> s_R = (pi / 2) * 1.2
    # In one small integration step, Runge-Kutta 2nd order approximates the arc:
    steps = 100
    phi_l, phi_r = 0.0, 0.0
    d_phi_l = ((math.pi / 2.0) * 0.8 / r) / steps
    d_phi_r = ((math.pi / 2.0) * 1.2 / r) / steps
    for _ in range(steps):
        phi_l += d_phi_l
        phi_r += d_phi_r
        x, y, yaw = odom.update((phi_l, phi_r))

    # Analytical end point of 90 deg left arc from origin: (R_c, R_c) = (1.0, 1.0)
    assert math.isclose(x, 1.0, rel_tol=0.01)
    assert math.isclose(y, 1.0, rel_tol=0.01)
    assert math.isclose(yaw, math.pi / 2.0, rel_tol=0.01)


def test_fused_pose():
    fused = FusedPose(wheel_radius=0.1)
    fused.initialize((0.0, 0.0, 0.0), (0.0, 0.0))

    # Both wheels advance by 10 rad -> 1.0 m, but IMU reports yaw = pi/2
    x, y, yaw = fused.update((0.0, 0.0, math.pi / 2), (10.0, 10.0))
    # When heading changes from 0 to pi/2, mid heading is pi/4 -> cos(pi/4)*1.0, sin(pi/4)*1.0
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-6)
    assert x > 0 and y > 0


def test_pose_manager_drift_tracking():
    pm = PoseManager(mode="odom")
    pm.initialize((0.0, 0.0, 0.0), (0.0, 0.0))

    # Suppose true robot moves to (1.0, 0.0, 0.0), but encoders only register 0.8 m
    # Encoders: 8 rad with r=0.0975 -> 0.78 m
    dphi = 0.8 / config.WHEEL_RADIUS
    est = pm.update(1.0, (1.0, 0.0, 0.0), (dphi, dphi))
    assert math.isclose(est[0], 0.8, abs_tol=1e-4)

    summary = pm.drift_summary()
    assert math.isclose(summary["final_pos_err"], 0.2, abs_tol=1e-3)
    assert math.isclose(summary["max_pos_err"], 0.2, abs_tol=1e-3)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
