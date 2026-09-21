"""Keep terrain rendering and physical placement aligned in the robot world."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from s1_navigation.terrain_world import create_robot_world

WORLDS = Path(__file__).resolve().parents[1] / 'worlds'


def test_robot_world_uses_terrain_constraint_and_full_odometry(tmp_path):
    path = create_robot_world(WORLDS, tmp_path / 'robot.sdf', sensors=False)
    world = ET.parse(path).find('world')
    robot = world.find("model[@name='robot']")
    assert robot is not None
    assert robot.findall('joint') == []
    base = robot.find("link[@name='base_link']")
    assert base.find('gravity').text == 'false'
    assert base.find('collision/geometry/box/size').text == '1.0 1.0 0.25'
    supports = [collision for collision in base.findall('collision')
                if collision.find('geometry/sphere') is not None]
    assert len(supports) == 4
    assert all(support.find('geometry/sphere/radius').text == '0.15'
               for support in supports)
    drive = robot.find("plugin[@name='s1::TerrainDrive']")
    assert drive.find('max_slope_deg').text == '30'
    assert Path(drive.find('heightmap_uri').text).is_absolute()
    assert Path(drive.find('heightmap_uri').text).is_file()
    assert robot.find("plugin[@name='gz::sim::systems::OdometryPublisher']/dimensions").text == '3'
    assert world.find("plugin[@name='gz::sim::systems::Sensors']") is None
    # All resources must still resolve after moving the SDF to a launch temp dir.
    for uri in world.findall('.//heightmap/uri'):
        assert Path(uri.text).is_absolute() and Path(uri.text).is_file()


def test_collision_and_visual_use_matching_engine_offsets(tmp_path):
    path = create_robot_world(WORLDS, tmp_path / 'robot.sdf')
    world = ET.parse(path).find('world')
    terrain = world.find("model[@name='usgs_utah_terrain']")
    import json
    info = json.loads((WORLDS / 'usgs_utah_metadata.json').read_text())
    assert terrain.find('pose').text == '0 0 0 0 0 0'
    collision = terrain.find('.//collision')
    visual = terrain.find('.//visual')
    # DART uses collision pose; Ogre terrain uses heightmap pos and ignores
    # ancestor visual/model transforms. Keep the parent at the origin.
    assert float(collision.find('pose').text.split()[2]) == pytest.approx(
        info['gazebo_z_offset_m'])
    assert collision.find('geometry/heightmap/pos').text == '0 0 0'
    assert float(
        visual.find('geometry/heightmap/pos').text.split()[2]
    ) == pytest.approx(info['gazebo_z_offset_m'])
    assert collision.find('geometry/heightmap/uri').text.endswith('usgs_utah_heightmap.png')
    assert visual.find('geometry/heightmap/uri').text.endswith(
        'usgs_utah_visual_heightmap.png')
    assert world.find('scene/shadows').text == 'false'
    assert world.find("plugin[@name='gz::sim::systems::Sensors']/render_engine").text == 'ogre'


def test_terrain_bases_have_half_buried_domes_and_connected_hallways(tmp_path):
    path = create_robot_world(WORLDS, tmp_path / 'robot.sdf', sensors=False,
                              base_seed=7)
    world = ET.parse(path).find('world')
    domes = [model for model in world.findall('model')
             if '_dome_' in model.get('name', '')]
    hallways = [model for model in world.findall('model')
                if '_hallway_' in model.get('name', '')]
    drive = world.find("model[@name='robot']/plugin[@name='s1::TerrainDrive']")

    assert 24 <= len(domes) <= 40
    assert len(domes) - 8 <= len(hallways) <= len(domes)
    # TerrainDrive receives the same geometry so kinematic pose updates stop
    # before the robot footprint enters the generated static collision models.
    assert len(drive.findall('dome')) == len(domes)
    assert len(drive.findall('hallway')) == len(hallways)
    central_domes = [model for model in domes
                     if model.get('name', '').endswith('_dome_1')]
    distances = [math.hypot(*map(float, model.find('pose').text.split()[:2]))
                 for model in central_domes]
    assert len(central_domes) == 8
    assert sum(100.0 <= distance <= 180.0 for distance in distances) == 5
    for dome in domes:
        radius = float(dome.find('.//visual/geometry/sphere/radius').text)
        collision_radius = float(dome.find('.//collision/geometry/sphere/radius').text)
        assert radius == collision_radius
        assert 8.0 <= radius <= 18.0
        # The model pose is the sphere centre, so terrain intersects its equator.
        assert math.isfinite(float(dome.find('pose').text.split()[2]))
    for hallway in hallways:
        assert hallway.find('.//visual/geometry/box/size').text == \
            hallway.find('.//collision/geometry/box/size').text
