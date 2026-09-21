"""Regression tests for following a path after passing its early points."""
from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import pytest

from s1_navigation.path_controller import PathController


def path(points):
    msg = Path()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        msg.poses.append(pose)
    return msg


def controller(points, x=0.0, y=0.0):
    node = SimpleNamespace(robot_x=x, robot_y=y, path_progress=0.0)
    node.select_target = MethodType(PathController.select_target, node)
    node.path_callback = MethodType(PathController.path_callback, node)
    node.path_callback(path(points))
    return node


def test_passed_start_does_not_pull_robot_backward():
    node = controller([(0, 0), (1, 0), (2, 0)], x=0.8)
    assert node.select_target() == pytest.approx((1.2, 0))


def test_progress_never_reverses_on_same_path():
    node = controller([(0, 0), (1, 0), (2, 0)], x=0.8)
    node.select_target()
    node.robot_x = 0.7
    assert node.select_target() == pytest.approx((1.2, 0))


def test_lookahead_follows_corner_arc_length():
    node = controller([(0, 0), (1, 0), (1, 1)], x=0.8)
    assert node.select_target() == pytest.approx((1, 0.2))


def test_replan_reprojects_instead_of_reusing_old_indices():
    node = controller([(0, 0), (1, 0), (2, 0)], x=1.5)
    node.select_target()
    node.path_callback(path([(1.4, 0), (1.6, 0), (2, 0)]))
    assert node.select_target() == pytest.approx((1.9, 0))


@pytest.mark.parametrize('points,expected', [
    ([], None), ([(1, 0)], (1, 0)),
    ([(0, 0), (0, 0), (1, 0)], (0.4, 0)),
    ([(0, 0), (0, 0)], (0, 0)),
])
def test_empty_single_and_duplicate_points(points, expected):
    node = controller(points)
    actual = node.select_target()
    assert actual == (pytest.approx(expected) if expected is not None else None)


def test_end_of_path_is_valid_target():
    node = controller([(0, 0), (1, 0)], x=1.1)
    assert node.select_target() == (1, 0)
    assert node.select_target() == (1, 0)


def test_close_obstacle_ahead_blocks_motion():
    node = SimpleNamespace(
        robot_x=0.0, robot_y=0.0,
        obstacle_stop_distance=3.0, corridor_half_width=1.0,
        obstacle_points=np.array([[2.0, 0.5], [-1.0, 0.0]]))
    assert PathController.obstacle_blocks_motion(node, 1.0, 0.0)


def test_obstacle_to_side_does_not_prevent_detour_motion():
    node = SimpleNamespace(
        robot_x=0.0, robot_y=0.0,
        obstacle_stop_distance=3.0, corridor_half_width=1.0,
        obstacle_points=np.array([[2.0, 0.0]]))
    assert not PathController.obstacle_blocks_motion(node, 0.0, 1.0)
