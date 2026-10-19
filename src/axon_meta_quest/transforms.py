"""Coordinate-frame helpers for Quest controller transforms (NumPy and SciPy).

See README.md "Coordinate Systems: ROS vs OpenXR" for the frame conventions.
Quaternions are (x, y, z, w). Euler angles are extrinsic x-y-z (roll, pitch,
yaw about the fixed axes), i.e. ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
"""

import numpy as np
from scipy.spatial.transform import Rotation

OPENXR_TO_ROS_QUAT = (0.5, -0.5, -0.5, 0.5)
"""Static (x, y, z, w) rotation from the OpenXR frame to the ROS frame."""

OPENXR_TO_ROS = np.eye(4)
OPENXR_TO_ROS[:3, :3] = Rotation.from_quat(OPENXR_TO_ROS_QUAT).as_matrix()


def openxr_to_ros(transform: np.ndarray) -> np.ndarray:
    """Express a 4x4 OpenXR-frame transform in the ROS frame."""
    return OPENXR_TO_ROS @ transform


def is_valid_transform(matrix: np.ndarray) -> bool:
    """Return whether ``matrix`` is a usable 4x4 rigid transform.

    The APK reports an all-zero matrix for an untracked controller.
    """
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        return False
    return bool(abs(np.linalg.det(matrix[:3, :3]) - 1.0) <= 0.1)


def matrix_to_pose(
    matrix: np.ndarray,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Split a 4x4 transform into a position and an (x, y, z, w) quaternion."""
    x, y, z = (float(v) for v in matrix[:3, 3])
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat(canonical=True)
    return (x, y, z), (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


def apply_axis_mask(transform: np.ndarray, axis_mask: np.ndarray) -> np.ndarray:
    """Zero the masked translation axes and Euler angles of ``transform``.

    Args:
        transform: 4x4 transform.
        axis_mask: Six 0/1 values for [x, y, z, roll, pitch, yaw] in the same
            frame as ``transform``; 0 zeroes that axis.

    Returns:
        The masked 4x4 transform.
    """
    masked = np.eye(4)
    masked[:3, 3] = transform[:3, 3] * axis_mask[:3]
    euler = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")
    masked[:3, :3] = Rotation.from_euler("xyz", euler * axis_mask[3:]).as_matrix()
    return masked
