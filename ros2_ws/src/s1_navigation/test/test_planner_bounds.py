"""The larger costmap must not permit paths outside the navigation region."""
from types import MethodType, SimpleNamespace

import pytest
from nav_msgs.msg import OccupancyGrid

from s1_navigation.astar_planner import AStarPlanner


def planner(resolution=1.0, width=30, origin=-15.0):
    grid = OccupancyGrid()
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = width
    grid.info.origin.position.x = origin
    grid.info.origin.position.y = origin
    grid.data = [0] * (width * width)
    node = SimpleNamespace(costmap=grid)
    for name in ('grid_to_world', 'world_to_grid', 'in_grid_bounds',
                 'astar', 'reconstruct_path', 'planning_callback'):
        setattr(node, name, MethodType(getattr(AStarPlanner, name), node))
    node.in_world_bounds = AStarPlanner.in_world_bounds
    return node


@pytest.mark.parametrize('point', [(-10.05, 0), (10.05, 0), (0, -10.05), (0, 10.05)])
def test_rejects_outside_cells(point):
    node = planner(0.1, 300)
    outside = node.world_to_grid(*point)
    inside = node.world_to_grid(0, 0)
    assert not node.in_grid_bounds(*outside)
    assert node.astar(outside, inside) is None
    assert node.astar(inside, outside) is None


def test_path_stays_inside_region():
    node = planner()
    path = node.astar(node.world_to_grid(-9.5, -9.5),
                      node.world_to_grid(9.5, 9.5))
    assert path
    assert all(node.in_world_bounds(*node.grid_to_world(*cell)) for cell in path)


def test_cannot_detour_outside_region():
    node = planner()
    # A wall covers the permitted region but leaves room outside it.
    for y in range(5, 25):
        node.costmap.data[y * 30 + 15] = 100
    assert node.astar((10, 15), (20, 15)) is None


def test_uses_actual_map_dimensions():
    node = planner(1.0, 4, -2.0)
    assert node.astar((0, 0), (3, 3))
    assert node.astar((-1, 0), (3, 3)) is None
    assert node.astar((0, 0), (4, 3)) is None


def test_outside_goal_clears_previous_path():
    from unittest.mock import Mock
    node = planner()
    node.have_odom = node.have_waypoints = True
    node.robot_x = node.robot_y = 0.0
    node.current_goal = Mock(return_value=SimpleNamespace(
        position=SimpleNamespace(x=11.0, y=0.0)))
    node.distance_to_goal = Mock(return_value=11.0)
    node.publish_path = Mock()
    node.planning_callback()
    node.publish_path.assert_called_once_with([])
