"""Check rotation conventions, round trips, and coordinate-frame helpers."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from axon_meta_quest import transforms as tf

ROTATIONS = list(Rotation.random(300, random_state=0)) + [
    Rotation.identity(),
    Rotation.from_rotvec([np.pi, 0, 0]),
    Rotation.from_rotvec([0, np.pi, 0]),
    Rotation.from_rotvec([0, 0, np.pi]),
    Rotation.from_euler("xyz", [0.3, np.pi / 2, 0.1]),
]


@pytest.mark.parametrize("rotation", ROTATIONS)
def test_matrix_to_pose_preserves_position_and_rotation(rotation: Rotation) -> None:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = [1.0, -2.0, 3.0]
    position, quat = tf.matrix_to_pose(matrix)
    assert position == (1.0, -2.0, 3.0)
    assert isinstance(quat, tuple)
    assert all(isinstance(value, float) for value in quat)
    assert quat[3] >= 0
    np.testing.assert_allclose(
        Rotation.from_quat(quat).as_matrix(), matrix[:3, :3], atol=1e-12
    )


def test_openxr_to_ros_matches_static_quaternion() -> None:
    expected = Rotation.from_quat([0.5, -0.5, -0.5, 0.5]).as_matrix()
    np.testing.assert_allclose(tf.OPENXR_TO_ROS[:3, :3], expected)
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, 3.0]  # right, up, backward in OpenXR
    # ROS: forward = -backward, left = -right, up = up.
    np.testing.assert_allclose(tf.openxr_to_ros(transform)[:3, 3], [-3.0, -1.0, 2.0])


def test_apply_axis_mask_zeroes_masked_axes() -> None:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix()
    transform[:3, 3] = [1.0, 2.0, 3.0]
    masked = tf.apply_axis_mask(transform, np.array([1, 0, 1, 0, 1, 1]))
    np.testing.assert_allclose(masked[:3, 3], [1.0, 0.0, 3.0])
    np.testing.assert_allclose(
        Rotation.from_matrix(masked[:3, :3]).as_euler("xyz"),
        [0.0, 0.2, 0.3],
        atol=1e-12,
    )


def test_is_valid_transform() -> None:
    assert tf.is_valid_transform(np.eye(4))
    assert not tf.is_valid_transform(np.zeros((4, 4)))
    assert not tf.is_valid_transform(np.eye(3))
    assert not tf.is_valid_transform(np.diag([2.0, 2.0, 2.0, 1.0]))


@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize("index", [(0, 0), (0, 3), (3, 0), (3, 3)])
def test_non_finite_transform_is_invalid(value: float, index: tuple[int, int]) -> None:
    matrix = np.eye(4)
    matrix[index] = value
    assert not tf.is_valid_transform(matrix)


def test_reflected_transform_is_invalid() -> None:
    assert not tf.is_valid_transform(np.diag([-1.0, 1.0, 1.0, 1.0]))
