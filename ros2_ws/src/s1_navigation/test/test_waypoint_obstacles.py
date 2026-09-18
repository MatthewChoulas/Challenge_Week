"""Check generated goals against the actual SDF collision geometry."""
import importlib.util
import json
import math
from pathlib import Path
import random
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

from s1_navigation import waypoint_publisher as wp

SCRIPT = Path(__file__).resolve().parents[1] / 'obstacle_generator.py'
spec = importlib.util.spec_from_file_location('obstacle_generator', SCRIPT)
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


def publisher(obstacles, seed=0):
    node = SimpleNamespace(obstacles=obstacles, rng=random.Random(seed),
                           get_logger=Mock(return_value=Mock()))
    for name in ('too_close_to_start', 'too_close_to_obstacle',
                 'too_close_to_waypoint', 'is_valid_waypoint', 'generate_waypoints'):
        setattr(node, name, MethodType(getattr(wp.WaypointPublisher, name), node))
    return node


def test_generator_paths_do_not_depend_on_working_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert generator.TEMPLATE_FILE.is_file()
    assert generator.OUTPUT_FILE.parent == SCRIPT.parent / 'worlds'
    assert generator.OBSTACLE_FILE.parent == SCRIPT.parent / 'worlds'


def test_random_worlds_and_waypoints_match_collision_geometry(tmp_path, monkeypatch):
    monkeypatch.setattr(generator, 'OUTPUT_FILE', tmp_path / 's1_world.sdf')
    monkeypatch.setattr(generator, 'OBSTACLE_FILE', tmp_path / 'obstacles.json')
    for seed in range(20):
        monkeypatch.setattr(generator, 'RANDOM_SEED', seed)
        generator.main(["--output-dir", str(tmp_path)])
        obstacles = wp.load_world_obstacles(generator.OUTPUT_FILE)
        assert obstacles == json.loads(generator.OBSTACLE_FILE.read_text())
        assert len(obstacles) == generator.NUM_OBSTACLES
        for goal_seed in range(20):
            goals = publisher(obstacles, goal_seed).generate_waypoints()
            assert len(goals) == wp.NUM_WAYPOINTS
            for i, (x, y) in enumerate(goals):
                assert -10 <= x <= 10 and -10 <= y <= 10
                assert math.hypot(x, y) >= wp.MIN_DISTANCE_FROM_START
                for obstacle in obstacles:
                    assert math.hypot(x - obstacle['x'], y - obstacle['y']) >= (
                        obstacle['radius'] + wp.ROBOT_RADIUS + wp.GOAL_OBSTACLE_CLEARANCE)
                for gx, gy in goals[:i]:
                    assert math.hypot(x - gx, y - gy) >= wp.MIN_DISTANCE_BETWEEN_GOALS


def test_inside_and_clearance_boundary():
    node = publisher([{'x': 3.0, 'y': 0.0, 'radius': 0.8}])
    assert not node.is_valid_waypoint(3.0, 0.0, [])
    assert not node.is_valid_waypoint(4.47, 0.0, [])
    assert node.is_valid_waypoint(4.49, 0.0, [])
    assert not node.is_valid_waypoint(11.0, 0.0, [])


def test_impossible_waypoints_fail_without_hanging(monkeypatch):
    monkeypatch.setattr(wp, 'MAX_SAMPLE_ATTEMPTS', 10)
    node = publisher([{'x': 0.0, 'y': 0.0, 'radius': 100.0}])
    with pytest.raises(RuntimeError, match='Unable to place waypoint'):
        node.generate_waypoints()


def test_impossible_obstacles_fail_without_hanging(tmp_path, monkeypatch):
    monkeypatch.setattr(generator, 'MAX_SAMPLE_ATTEMPTS', 10)
    monkeypatch.setattr(generator, 'ROBOT_CLEARANCE', 100.0)
    monkeypatch.setattr(generator, 'OBSTACLE_FILE', tmp_path / 'obstacles.json')
    with pytest.raises(RuntimeError, match='Unable to place obstacle'):
        generator.generate_obstacles()
    assert not generator.OBSTACLE_FILE.exists()


def test_malformed_rock_is_not_silently_ignored(tmp_path):
    world = tmp_path / 'invalid.sdf'
    world.write_text('<sdf><world><model name="rock_0"><pose>3 0 0 0 0 0</pose>'
                     '</model></world></sdf>')
    with pytest.raises(ValueError, match='Invalid generated obstacle'):
        wp.load_world_obstacles(world)
