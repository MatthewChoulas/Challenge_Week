"""3D lidar coordinate transform behavior."""
import math

import numpy as np

from s1_navigation.lidar_mapper import (
    FREE_SPACE_UPDATE,
    interpolate_odom_pose,
    is_goal_post_return,
    LidarMapper,
    MIN_HIT_OBSERVATIONS,
    OCCUPIED_PROBABILITY,
    OCCUPIED_SPACE_UPDATE,
    probability_from_log_odds,
    quaternion_rotation_matrix,
    ray_voxel_keys,
)


def test_quaternion_rotation_matrix_rotates_full_3d_point():
    yaw = math.pi / 2.0
    rotation = quaternion_rotation_matrix((0.0, 0.0,
                                           math.sin(yaw / 2.0),
                                           math.cos(yaw / 2.0)))
    np.testing.assert_allclose(rotation @ np.array([1.0, 0.0, 0.0]),
                               [0.0, 1.0, 0.0], atol=1e-7)


def test_zero_quaternion_is_safe_identity():
    np.testing.assert_array_equal(
        quaternion_rotation_matrix((0.0, 0.0, 0.0, 0.0)), np.eye(3))


def test_3d_ray_contains_free_voxels_before_endpoint():
    keys = ray_voxel_keys((0.1, 0.1, 0.1), (1.1, 0.1, 0.6), 0.25)
    assert keys[0] == (0, 0, 0)
    assert keys[-1] == (4, 0, 2)
    assert len(keys) > 2


def test_bayesian_hit_and_free_updates_change_probability():
    mapper = type('Mapper', (), {'voxels': {}})()
    key = (1, 2, 3)

    LidarMapper.update_voxel(mapper, key, OCCUPIED_SPACE_UPDATE, 1, hit_scan=1)
    assert not LidarMapper.confirmed_occupied(mapper.voxels[key])

    LidarMapper.update_voxel(mapper, key, OCCUPIED_SPACE_UPDATE, 2, hit_scan=2)
    assert mapper.voxels[key][2] == MIN_HIT_OBSERVATIONS
    assert probability_from_log_odds(mapper.voxels[key][0]) >= OCCUPIED_PROBABILITY
    assert LidarMapper.confirmed_occupied(mapper.voxels[key])

    for stamp in range(3, 12):
        LidarMapper.update_voxel(mapper, key, FREE_SPACE_UPDATE, stamp)
    assert probability_from_log_odds(mapper.voxels[key][0]) < 0.5
    assert mapper.voxels[key][2] == 0


def test_cloud_pose_is_interpolated_at_its_timestamp():
    history = [
        (1_000_000_000, np.array([0.0, 0.0, 0.0]),
         np.array([0.0, 0.0, 0.0, 1.0])),
        (2_000_000_000, np.array([10.0, 0.0, 0.0]),
         np.array([0.0, 0.0, 1.0, 0.0])),
    ]
    position, quaternion = interpolate_odom_pose(history, 1_500_000_000)
    np.testing.assert_allclose(position, [5.0, 0.0, 0.0])
    np.testing.assert_allclose(np.linalg.norm(quaternion), 1.0)


def test_goal_post_returns_are_excluded_from_obstacle_mapping():
    posts = np.array([[10.0, -5.0]], dtype=np.float32)
    assert is_goal_post_return(10.8, -5.0, posts)
    assert not is_goal_post_return(11.3, -5.0, posts)
    assert not is_goal_post_return(10.0, -5.0, np.empty((0, 2)))


def test_global_obstacle_has_hard_center_and_graded_clearance():
    mapper = type('Mapper', (), {
        'global_clearance': 3.0,
        'global_resolution': 1.0,
        'global_width': 9,
        'global_grid': np.full((9, 9), -1, dtype=np.int8),
    })()
    LidarMapper.add_global_obstacle(mapper, 4, 4)
    assert mapper.global_grid[4, 4] == 100
    assert 0 < mapper.global_grid[4, 5] < 100
    assert mapper.global_grid[4, 5] > mapper.global_grid[4, 6]
    assert mapper.global_grid[0, 0] == -1
