"""Terrain A* bounds and cost behavior."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

from s1_navigation.astar_planner import AStarPlanner


def planner(resolution=1.0, width=30, origin=-15.0):
    node = SimpleNamespace(
        terrain_cost=np.zeros((width, width), dtype=np.float32),
        elevation=np.zeros((width, width), dtype=np.float32),
        resolution=resolution, width=width, height=width,
        origin_x=origin, origin_y=origin)
    for name in ('grid_to_world', 'world_to_grid', 'in_grid_bounds',
                 'in_world_bounds', 'astar', 'path_cell_free',
                 'path_section_clear', 'remaining_path_index', 'repair_path'):
        setattr(node, name, MethodType(getattr(AStarPlanner, name), node))
    node.reconstruct_path = AStarPlanner.reconstruct_path
    return node


@pytest.mark.parametrize('point', [(-15.05, 0), (15.05, 0),
                                   (0, -15.05), (0, 15.05)])
def test_rejects_outside_cells(point):
    node = planner(0.1, 300)
    outside = node.world_to_grid(*point)
    inside = node.world_to_grid(0, 0)
    assert not node.in_grid_bounds(*outside)
    assert node.astar(outside, inside) is None
    assert node.astar(inside, outside) is None


def test_path_uses_full_grid():
    node = planner()
    path = node.astar(node.world_to_grid(-14.5, -14.5),
                      node.world_to_grid(14.5, 14.5))
    assert path
    assert all(node.in_world_bounds(*node.grid_to_world(*cell)) for cell in path)


def test_thirty_degree_wall_is_impassable():
    node = planner()
    node.terrain_cost[:, 15] = 1.0
    assert node.astar((10, 15), (20, 15)) is None


def test_blocked_start_and_goal_are_rejected():
    node = planner(width=5, origin=0.0)
    node.terrain_cost[0, 0] = 1.0
    assert node.astar((0, 0), (4, 4)) is None
    node.terrain_cost[0, 0] = 0.0
    node.terrain_cost[4, 4] = 1.0
    assert node.astar((0, 0), (4, 4)) is None


def test_diagonal_cannot_cut_between_blocked_cells():
    node = planner(width=2, origin=0.0)
    node.terrain_cost[0, 1] = 1.0
    node.terrain_cost[1, 0] = 1.0
    assert node.astar((0, 0), (1, 1)) is None


def test_gradual_cost_avoids_expensive_terrain():
    node = planner(width=7, origin=0.0)
    node.terrain_cost[3, 1:6] = 0.9
    path = node.astar((0, 3), (6, 3))
    assert path
    assert any(y != 3 for _, y in path[1:-1])


def test_global_obstacle_cells_are_impassable():
    node = planner(width=7, origin=0.0)
    node.obstacle_cost = np.zeros((7, 7), dtype=np.float32)
    node.obstacle_cost[:, 3] = 1.0
    assert node.astar((1, 3), (5, 3)) is None


def test_global_clearance_cost_pushes_path_away_from_obstacle():
    node = planner(width=9, origin=0.0)
    node.obstacle_cost = np.zeros((9, 9), dtype=np.float32)
    node.obstacle_cost[4, 2:7] = 0.9
    path = node.astar((0, 4), (8, 4))
    assert path
    assert any(y != 4 for _, y in path[1:-1])


def test_uses_actual_map_dimensions():
    node = planner(1.0, 4, -2.0)
    assert node.astar((0, 0), (3, 3))
    assert node.astar((-1, 0), (3, 3)) is None
    assert node.astar((0, 0), (4, 3)) is None


def test_unreachable_goal_is_reported_cleared_and_skipped():
    messages = []
    logger = SimpleNamespace(
        warning=lambda message: messages.append(('warning', message)),
        info=lambda message: messages.append(('info', message)))
    goals = [SimpleNamespace(position=SimpleNamespace(x=4.0, y=5.0)),
             SimpleNamespace(position=SimpleNamespace(x=8.0, y=9.0))]
    node = SimpleNamespace(
        waypoints=goals, have_waypoints=True, current_waypoint_index=0,
        last_plan_position=(1.0, 2.0), last_planned_goal_index=0,
        published_paths=[], get_logger=lambda: logger)
    node.current_goal = MethodType(AStarPlanner.current_goal, node)
    node.publish_path = lambda path: node.published_paths.append(path)
    node.skip_current_goal = MethodType(AStarPlanner.skip_current_goal, node)

    node.skip_current_goal('no traversable terrain path exists')

    assert node.current_waypoint_index == 1
    assert node.current_goal() is goals[1]
    assert node.published_paths == [[]]
    assert node.last_plan_position is None
    assert node.last_planned_goal_index is None
    assert messages == [('warning',
                         'Skipping waypoint 1 at (4.00, 5.00): '
                         'no traversable terrain path exists')]


def test_bounded_search_cannot_escape_window():
    node = planner(width=80, origin=0.0)
    node.terrain_cost[:21, 10] = 1.0
    assert node.astar((5, 10), (15, 10), bounds=(0, 0, 20, 20)) is None
    assert node.astar((5, 10), (15, 10))


def test_partial_repair_avoids_obstacle_and_preserves_suffix():
    node = planner(width=100, origin=0.0)
    node.active_path = [(x, 50) for x in range(5, 95)]
    node.obstacle_cost = np.zeros((100, 100), dtype=np.float32)
    node.obstacle_cost[48:53, 15] = 1.0
    repaired = node.repair_path((5, 50), 0)
    assert repaired
    assert repaired[0] == (5, 50)
    assert repaired[-69:] == node.active_path[-69:]
    assert node.path_section_clear(repaired)
    assert any(y != 50 for _, y in repaired)


def test_failed_repairs_return_none_for_global_fallback():
    node = planner(width=100, origin=0.0)
    node.active_path = [(x, 50) for x in range(5, 95)]
    node.terrain_cost[:, 15] = 1.0
    assert node.repair_path((5, 50), 0) is None


def test_repair_expands_past_blocked_first_rejoin():
    node = planner(width=100, origin=0.0)
    node.active_path = [(x, 50) for x in range(5, 95)]
    node.terrain_cost[50, 25] = 1.0
    repaired = node.repair_path((5, 50), 0)
    assert repaired
    assert node.path_section_clear(repaired)
    assert repaired[-49:] == node.active_path[-49:]
