"""Compose the terrain and free-body robot without modifying downloaded assets."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from s1_navigation.terrain_bases import generate_base_models, HALLWAY_WIDTH


def configure_terrain_drive_obstacles(robot, layouts):
    """Give TerrainDrive the same 2D base footprints used by Gazebo visuals."""
    drive = robot.find("plugin[@name='s1::TerrainDrive']")
    if drive is None:
        raise RuntimeError('terrain robot is missing the TerrainDrive plugin')
    for cluster in layouts:
        for x, y, radius in cluster['domes']:
            dome = ET.SubElement(drive, 'dome')
            ET.SubElement(dome, 'x').text = f'{x:.4f}'
            ET.SubElement(dome, 'y').text = f'{y:.4f}'
            ET.SubElement(dome, 'radius').text = f'{radius:.4f}'
        for first, second in cluster['connections']:
            start, end = cluster['domes'][first], cluster['domes'][second]
            dx, dy = end[0] - start[0], end[1] - start[1]
            hallway = ET.SubElement(drive, 'hallway')
            ET.SubElement(hallway, 'x').text = f'{(start[0] + end[0]) / 2.0:.4f}'
            ET.SubElement(hallway, 'y').text = f'{(start[1] + end[1]) / 2.0:.4f}'
            ET.SubElement(hallway, 'yaw').text = f'{math.atan2(dy, dx):.6f}'
            ET.SubElement(hallway, 'length').text = f'{math.hypot(dx, dy):.4f}'
            ET.SubElement(hallway, 'width').text = f'{HALLWAY_WIDTH:.4f}'


def create_robot_world(worlds_dir, destination, *, sensors=True, base_seed=None):
    worlds_dir = Path(worlds_dir)
    tree = ET.parse(worlds_dir / 'usgs_utah.sdf')
    world = tree.getroot().find('world')
    robot = ET.parse(worlds_dir / 'terrain_robot.sdf').getroot().find('model')
    world.append(robot)
    layouts = generate_base_models(worlds_dir, world, seed=base_seed)
    configure_terrain_drive_obstacles(robot, layouts)
    if sensors:
        plugin = ET.SubElement(world, 'plugin', filename='gz-sim-sensors-system',
                               name='gz::sim::systems::Sensors')
        ET.SubElement(plugin, 'render_engine').text = 'ogre'
    # Resolve assets before moving the composed world into a temporary directory.
    for tag in ('uri', 'diffuse', 'normal', 'heightmap_uri'):
        for element in world.iter(tag):
            value = (element.text or '').strip()
            if value and (worlds_dir / value).is_file():
                element.text = str((worlds_dir / value).resolve())
    camera = world.find("gui/plugin[@filename='MinimalScene']/camera_pose")
    camera.text = '4 -6 4 0 0.5 2.1588'
    ET.indent(tree)
    tree.write(destination, encoding='unicode', xml_declaration=True)
    return str(destination)
