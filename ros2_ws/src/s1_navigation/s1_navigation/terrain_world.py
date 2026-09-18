"""Compose the terrain and free-body robot without modifying downloaded assets."""
from pathlib import Path
import xml.etree.ElementTree as ET


def create_robot_world(worlds_dir, destination, *, sensors=True):
    worlds_dir = Path(worlds_dir)
    tree = ET.parse(worlds_dir / 'usgs_utah.sdf')
    world = tree.getroot().find('world')
    robot = ET.parse(worlds_dir / 'terrain_robot.sdf').getroot().find('model')
    world.append(robot)
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
