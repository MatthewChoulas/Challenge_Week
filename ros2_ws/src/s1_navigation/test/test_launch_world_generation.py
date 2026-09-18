"""Verify launch generation completes before starting consumers of the world."""
import importlib.util
from pathlib import Path
import subprocess
from unittest.mock import Mock

from launch import LaunchContext
import pytest

from s1_navigation.waypoint_publisher import load_world_obstacles


@pytest.fixture
def simulation(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch/simulation.launch.py'
    spec = importlib.util.spec_from_file_location('simulation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', Mock())
    monkeypatch.setattr(module, 'ExecuteProcess', Mock())
    return module


def test_generated_world_is_shared_by_gazebo_and_waypoints(simulation, tmp_path, monkeypatch):
    monkeypatch.setattr(simulation.tempfile, 'mkdtemp', lambda **kwargs: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations['regenerate_obstacles'] = 'true'
    simulation.launch_simulation(context)
    world = tmp_path / 's1_world.sdf'
    assert len(load_world_obstacles(world)) == 30
    server = simulation.ExecuteProcess.call_args_list[0].kwargs
    assert str(world) in server['cmd']
    assert '-s' in server['cmd']
    assert 'on_exit' in server
    gui = simulation.ExecuteProcess.call_args_list[1].kwargs
    assert gui['cmd'][-1].endswith('/bin/s1_gazebo_gui')
    assert 'on_exit' in gui
    waypoint = next(call.kwargs for call in simulation.Node.call_args_list
                    if call.kwargs['name'] == 'waypoint_publisher')
    assert waypoint['parameters'][0]['world_file'] == str(world)


def test_generation_failure_prevents_simulation_start(simulation, monkeypatch):
    def fail(*args, **kwargs):
        raise subprocess.CalledProcessError(1, 'generator')
    monkeypatch.setattr(simulation.subprocess, 'run', fail)
    context = LaunchContext()
    context.launch_configurations['regenerate_obstacles'] = 'true'
    with pytest.raises(subprocess.CalledProcessError):
        simulation.launch_simulation(context)
    simulation.Node.assert_not_called()
    simulation.ExecuteProcess.assert_not_called()


def test_generation_can_be_disabled(simulation, monkeypatch):
    generate = Mock()
    monkeypatch.setattr(simulation.subprocess, 'run', generate)
    context = LaunchContext()
    context.launch_configurations['regenerate_obstacles'] = 'false'
    simulation.launch_simulation(context)
    generate.assert_not_called()
