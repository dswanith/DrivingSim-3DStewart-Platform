"""
sil_sim.motion_cueing.frame_transform — World-frame to body-frame rotation.

CARLA's get_acceleration(), get_velocity(), and get_angular_velocity() all
return vectors in WORLD (global) coordinates, not the vehicle's local body
frame. The MCA pipeline requires body-frame quantities (forward = +X,
left = +Y, up = +Z) for correct tilt coordination and translational cueing.

This module provides the rotation step that converts world-frame vectors
into body-frame vectors using the vehicle's own orientation.

The rotation matrix is sourced from sil_sim.kinematics.geometry (the same
build_rotation_matrix_zyx used by the IK solver) — one source of truth
for the Euler convention used throughout this codebase.

Rotation direction:
    R(roll, pitch, yaw)  maps  body → world
    R^T(roll, pitch, yaw) maps  world → body

We need world → body, so we apply R^T (the transpose, which for a rotation
matrix equals the inverse).

Output class: EXACT — pure coordinate rotation, no model assumptions.
"""

import math
import numpy as np
from typing import Tuple

from sil_sim.kinematics.geometry import build_rotation_matrix_zyx


def rotate_world_to_body_full(
    accel_world: np.ndarray,
    ang_vel_world: np.ndarray,
    orientation_rpy: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Rotate acceleration and angular velocity from world frame to body frame
    using the full 3D rotation matrix R^T(roll, pitch, yaw).

    Args:
        accel_world: (3,) world-frame acceleration [ax, ay, az] (m/s²).
        ang_vel_world: (3,) world-frame angular velocity [wx, wy, wz] (rad/s).
        orientation_rpy: (3,) vehicle orientation [roll, pitch, yaw] (rad).

    Returns:
        Tuple of:
            - (3,) body-frame acceleration [ax, ay, az] (m/s²)
            - (3,) body-frame angular velocity [wx, wy, wz] (rad/s)
    """
    roll, pitch, yaw = orientation_rpy[0], orientation_rpy[1], orientation_rpy[2]

    # R maps body → world; R^T maps world → body
    R = build_rotation_matrix_zyx(roll, pitch, yaw)
    R_T = R.T

    accel_body = R_T @ accel_world
    ang_vel_body = R_T @ ang_vel_world

    return accel_body, ang_vel_body


def rotate_world_to_body_yaw(
    accel_world: np.ndarray,
    ang_vel_world: np.ndarray,
    yaw: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Rotate acceleration and angular velocity from world frame to body frame
    using a yaw-only 2D rotation (X/Y plane only, Z untouched).

    This is a simplified version that ignores roll and pitch — adequate for
    flat driving but incorrect once pitch under braking or roll under
    cornering become significant.

    Provided as a debug/fallback option, NOT the default.

    The 2D rotation for yaw-only world→body is:
        body_x =  cos(yaw) * world_x + sin(yaw) * world_y
        body_y = -sin(yaw) * world_x + cos(yaw) * world_y
        body_z =  world_z

    This is the transpose of the 2D yaw rotation matrix:
        Rz(yaw) = [[cos, -sin], [sin, cos]]
        Rz^T    = [[cos,  sin], [-sin, cos]]

    Args:
        accel_world: (3,) world-frame acceleration [ax, ay, az] (m/s²).
        ang_vel_world: (3,) world-frame angular velocity [wx, wy, wz] (rad/s).
        yaw: Vehicle yaw angle (rad).

    Returns:
        Tuple of:
            - (3,) body-frame acceleration [ax, ay, az] (m/s²)
            - (3,) body-frame angular velocity [wx, wy, wz] (rad/s)
    """
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    accel_body = np.array([
        cy * accel_world[0] + sy * accel_world[1],
        -sy * accel_world[0] + cy * accel_world[1],
        accel_world[2],
    ])

    ang_vel_body = np.array([
        cy * ang_vel_world[0] + sy * ang_vel_world[1],
        -sy * ang_vel_world[0] + cy * ang_vel_world[1],
        ang_vel_world[2],
    ])

    return accel_body, ang_vel_body


def rotate_world_to_body(
    accel_world: np.ndarray,
    ang_vel_world: np.ndarray,
    orientation_rpy: np.ndarray,
    mode: str = "full",
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Dispatch function: rotate world-frame vectors to body frame.

    Args:
        accel_world: (3,) world-frame acceleration (m/s²).
        ang_vel_world: (3,) world-frame angular velocity (rad/s).
        orientation_rpy: (3,) vehicle orientation [roll, pitch, yaw] (rad).
        mode: "full" for 3D R^T rotation (default),
              "yaw_only" for 2D yaw-only rotation (debug/fallback).

    Returns:
        Tuple of (accel_body, ang_vel_body), each (3,) arrays.

    Raises:
        ValueError: If mode is not "full" or "yaw_only".
    """
    if mode == "full":
        return rotate_world_to_body_full(accel_world, ang_vel_world, orientation_rpy)
    elif mode == "yaw_only":
        return rotate_world_to_body_yaw(
            accel_world, ang_vel_world, orientation_rpy[2]
        )
    else:
        raise ValueError(
            "Unknown body_frame_rotation mode: '{}'. "
            "Expected 'full' or 'yaw_only'.".format(mode)
        )
