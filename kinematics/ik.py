"""
sil_sim.kinematics.ik — Stewart-platform Inverse Kinematics.

Thin wrapper around StewartGeometry.compute_leg_lengths() for pipeline use.
Kept as a separate module so the IK interface is clean and independently testable.

Equation (§5):
    L_i = ‖T + R(roll, pitch, yaw) · p_i − b_i‖

Output class: EXACT — pure geometry, no model assumptions.
"""

import numpy as np
from typing import Tuple

from sil_sim.kinematics.geometry import StewartGeometry


def inverse_kinematics(
    pose: np.ndarray,
    geometry: StewartGeometry,
) -> np.ndarray:
    """
    Compute 6 leg lengths from a 6-DOF platform pose.

    IK equation (from §5, cited here):
        L_i = ‖T + R(roll, pitch, yaw) · p_i − b_i‖
    where:
        T = [x, y, z]   — translation of platform center from base origin
        R              — rotation matrix from (roll, pitch, yaw)
        p_i            — platform joint i in platform-local coordinates
        b_i            — base joint i in base coordinates

    Output class: EXACT.

    Args:
        pose: (6,) array [x, y, z, roll, pitch, yaw] in SI (m, rad).
        geometry: StewartGeometry instance with joint positions loaded.

    Returns:
        (6,) array of leg lengths [L1..L6] in metres.
    """
    return geometry.compute_leg_lengths(pose)


def inverse_kinematics_with_vectors(
    pose: np.ndarray,
    geometry: StewartGeometry,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute 6 leg lengths and leg vectors from a 6-DOF platform pose.

    Args:
        pose: (6,) array [x, y, z, roll, pitch, yaw] in SI (m, rad).
        geometry: StewartGeometry instance.

    Returns:
        Tuple of:
            - (6,) array of leg lengths in metres
            - (6, 3) array of leg vectors (base→platform)
    """
    leg_vectors = geometry.compute_leg_vectors(pose)
    leg_lengths = np.linalg.norm(leg_vectors, axis=1)
    return leg_lengths, leg_vectors
