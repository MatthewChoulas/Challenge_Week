"""Regression coverage for concurrent simulation startup."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped

from s1_navigation.astar_planner import AStarPlanner
from s1_navigation.lidar_mapper import LidarMapper
from s1_navigation.path_controller import PathController


def test_mapper_waits_for_odometry_and_scan():
    mapper = SimpleNamespace(have_odom=False, have_scan=False)
    # Neither callback may touch map state before its input is ready.
    LidarMapper.scan_callback(mapper, SimpleNamespace(ranges=[1.0]))
    LidarMapper.publish_map(mapper)
    mapper.have_odom = True
    LidarMapper.scan_callback(mapper, SimpleNamespace(ranges=[]))
    LidarMapper.publish_map(mapper)


def test_planner_waits_for_odometry():
    planner = SimpleNamespace(costmap=object(), have_odom=False)
    AStarPlanner.planning_callback(planner)


def controller(have_odom=True):
    goal = PoseStamped()
    goal.pose.position.x = 10.0
    return SimpleNamespace(
        have_odom=have_odom, path=[goal], robot_x=0.0, robot_y=0.0,
        robot_yaw=0.0, max_speed=0.5, stop_robot=Mock(),
        select_target=Mock(return_value=(10.0, 10.0)),
        cmd_vel_publisher=Mock(),
    )


def test_controller_waits_for_odometry():
    node = controller(have_odom=False)
    PathController.control_callback(node)
    node.stop_robot.assert_called_once()
    node.cmd_vel_publisher.publish.assert_not_called()


def test_controller_caps_total_speed():
    node = controller()
    PathController.control_callback(node)
    cmd = node.cmd_vel_publisher.publish.call_args.args[0]
    assert (cmd.linear.x ** 2 + cmd.linear.y ** 2) ** 0.5 == pytest.approx(0.5)


def test_controller_handles_target_at_robot():
    node = controller()
    node.select_target.return_value = (0.0, 0.0)
    PathController.control_callback(node)
    node.stop_robot.assert_called_once()
