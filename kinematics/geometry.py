"""
sil_sim.kinematics.geometry — Stewart-platform geometry model.

Computes base and platform joint Cartesian coordinates from the parameterized
config (clocking angles + bolt-circle radii). Builds the rotation matrix
using the configured Euler convention.

All positions are in SI units (metres). Angles internally in radians.

Output class: EXACT — pure geometry, no model assumptions.
"""

import math
import numpy as np
from typing import Dict, Any, List, Tuple


def build_rotation_matrix_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Build a 3×3 rotation matrix from roll, pitch, yaw angles.

    Convention: Intrinsic Z-Y-X (yaw → pitch → roll), right-handed
    automotive frame.
        R = Rz(yaw) · Ry(pitch) · Rx(roll)

    This is the standard aerospace/automotive convention where:
        - X points forward
        - Y points left
        - Z points up
        - Rotations are applied intrinsically: first yaw about Z,
          then pitch about new Y, then roll about new X.

    This is a MODULE-LEVEL function so it can be imported by other modules
    (e.g. frame_transform.py) without needing a StewartGeometry instance.
    Single source of truth for the rotation convention used in this codebase.

    Args:
        roll: Roll angle (radians), rotation about X.
        pitch: Pitch angle (radians), rotation about Y.
        yaw: Yaw angle (radians), rotation about Z.

    Returns:
        (3, 3) rotation matrix.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R = Rz(yaw) · Ry(pitch) · Rx(roll)
    R = np.array([
        [cy * cp,   cy * sp * sr - sy * cr,   cy * sp * cr + sy * sr],
        [sy * cp,   sy * sp * sr + cy * cr,   sy * sp * cr - cy * sr],
        [-sp,       cp * sr,                   cp * cr               ],
    ])
    return R


class StewartGeometry:
    """
    Stewart-platform geometry: computes and stores base and platform
    joint positions from parameterized config.

    Joint layout:
        - 3 pairs of joints on each plate
        - Each pair defined by two clocking angles on a bolt-circle
        - Pairs are NOT required to be evenly spaced (though the default is)

    The ordering of joints matters: base joint i connects to platform joint i
    via leg i. The standard 6-3 Stewart cross-coupling is achieved by
    offsetting the platform pair angles from the base pair angles.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize geometry from a geometry config dict (from loader.load_geometry()).

        Args:
            config: Validated geometry config dict.
        """
        self.euler_convention = config['euler_convention']

        # --- Base joints ---
        # Append z=0 to each (x, y) pair
        base_xy = config['base']['joints_xy_m']
        self.base_joints = np.array([[xy[0], xy[1], 0.0] for xy in base_xy])

        # --- Platform joints (in platform-local frame, origin at platform center) ---
        plat_xy = config['platform']['joints_xy_m']
        self.platform_joints = np.array([[xy[0], xy[1], 0.0] for xy in plat_xy])

        # --- Neutral pose ---
        self.neutral_height = config['neutral_height_m']  # metres

        # --- Joint limits ---
        hw_limit = config['joint_hardware_limit_deg']
        margin = config['joint_operating_margin_deg']
        self.joint_max_angle_rad = math.radians(hw_limit - margin)

        # --- Workspace limits (convert to radians internally) ---
        wl = config['workspace_limits']
        self.workspace_limits = {
            'max_tx': wl['max_translation_x_m'],
            'max_ty': wl['max_translation_y_m'],
            'max_tz': wl['max_translation_z_m'],
            'max_roll': math.radians(wl['max_roll_deg']),
            'max_pitch': math.radians(wl['max_pitch_deg']),
            'max_yaw': math.radians(wl['max_yaw_deg']),
        }

        # Store config for reference
        self.config = config

        # Compute home (neutral) leg lengths — EXACT
        home_pose = np.array([0.0, 0.0, self.neutral_height, 0.0, 0.0, 0.0])
        self.home_leg_lengths = self.compute_leg_lengths(home_pose)



    def rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Build a 3×3 rotation matrix from roll, pitch, yaw angles.

        Delegates to the module-level build_rotation_matrix_zyx() function.
        See that function's docstring for convention details.

        Args:
            roll: Roll angle (radians), rotation about X.
            pitch: Pitch angle (radians), rotation about Y.
            yaw: Yaw angle (radians), rotation about Z.

        Returns:
            (3, 3) rotation matrix.
        """
        if self.euler_convention != "ZYX_intrinsic":
            raise NotImplementedError(
                "Only ZYX_intrinsic Euler convention is implemented. "
                "Got: {}".format(self.euler_convention)
            )

        return build_rotation_matrix_zyx(roll, pitch, yaw)

    def compute_leg_lengths(self, pose: np.ndarray) -> np.ndarray:
        """
        Compute all 6 leg lengths for a given platform pose.

        Inverse Kinematics equation (§5):
            L_i = ‖T + R(roll, pitch, yaw) · p_i − b_i‖

        where:
            T = [x, y, z] translation of platform center
            R = rotation matrix from roll, pitch, yaw
            p_i = platform joint i in platform-local coords
            b_i = base joint i in base coords

        Output class: EXACT — pure geometry.

        Args:
            pose: [x, y, z, roll, pitch, yaw] in SI units (m, rad).

        Returns:
            (6,) ndarray of leg lengths in metres.
        """
        T = pose[:3]                          # Translation vector [x, y, z]
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        R = self.rotation_matrix(roll, pitch, yaw)

        leg_lengths = np.zeros(6)
        for i in range(6):
            # IK: L_i = ‖T + R · p_i − b_i‖
            leg_vec = T + R @ self.platform_joints[i] - self.base_joints[i]
            leg_lengths[i] = np.linalg.norm(leg_vec)

        return leg_lengths

    def compute_leg_vectors(self, pose: np.ndarray) -> np.ndarray:
        """
        Compute all 6 leg vectors (base joint → platform joint in world frame).

        Args:
            pose: [x, y, z, roll, pitch, yaw] in SI units.

        Returns:
            (6, 3) ndarray of leg vectors.
        """
        T = pose[:3]
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        R = self.rotation_matrix(roll, pitch, yaw)

        leg_vectors = np.zeros((6, 3))
        for i in range(6):
            leg_vectors[i] = T + R @ self.platform_joints[i] - self.base_joints[i]

        return leg_vectors

    def compute_joint_angles(self, pose: np.ndarray) -> np.ndarray:
        """
        Compute the angle each leg makes with the vertical (Z-axis)
        at both the base and platform joints.

        Used for workspace validation (rod-end articulation limit = ±25°).

        Args:
            pose: [x, y, z, roll, pitch, yaw] in SI units.

        Returns:
            (6,) ndarray of angles in radians (max of base/platform angle per leg).
        """
        leg_vectors = self.compute_leg_vectors(pose)
        leg_lengths = np.linalg.norm(leg_vectors, axis=1)

        # Angle with Z-axis: cos(θ) = leg_z / |leg|
        z_unit = np.array([0.0, 0.0, 1.0])
        angles = np.zeros(6)
        for i in range(6):
            if leg_lengths[i] > 1e-10:
                cos_angle = np.dot(leg_vectors[i], z_unit) / leg_lengths[i]
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angles[i] = math.acos(cos_angle)
            else:
                angles[i] = 0.0

        return angles
