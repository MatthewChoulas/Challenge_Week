"""Generate simple, terrain-aware dome and hallway models for Gazebo."""
import math
import random
import xml.etree.ElementTree as ET

from PIL import Image

# Edit this value to change how many bases are added to the terrain world.
NUM_BASE_CLUSTERS = 8
MIN_DOMES_PER_CLUSTER = 3
MAX_DOMES_PER_CLUSTER = 5
NEAR_ORIGIN_CLUSTERS = 3
NEAR_ORIGIN_MIN_DISTANCE = 100.0
NEAR_ORIGIN_MAX_DISTANCE = 180.0
NEAR_CLUSTER_SPACING = 90.0
FAR_CLUSTER_SPACING = 180.0
HALLWAY_WIDTH = 6.0
HALLWAY_HEIGHT = 5.0


class TerrainSampler:
    """Sample the same heightmap, scale, and offset used by Gazebo."""

    def __init__(self, worlds_dir, heightmap):
        self.image = Image.open(worlds_dir / heightmap.findtext('uri'))
        self.size, _, self.height_range = map(float, heightmap.findtext('size').split())
        self.z_offset = float(heightmap.findtext('pos').split()[2])

    def height(self, x, y):
        px = min(max((x / self.size + 0.5) * (self.image.width - 1), 0),
                 self.image.width - 1)
        # Image row zero is the north (+Y) edge.
        py = min(max((0.5 - y / self.size) * (self.image.height - 1), 0),
                 self.image.height - 1)
        ix, iy = math.floor(px), math.floor(py)
        nx = min(ix + 1, self.image.width - 1)
        ny = min(iy + 1, self.image.height - 1)
        tx, ty = px - ix, py - iy

        def value(column, row):
            return self.image.getpixel((column, row)) / 65535.0

        normalized = (
            (1 - tx) * (1 - ty) * value(ix, iy)
            + tx * (1 - ty) * value(nx, iy)
            + (1 - tx) * ty * value(ix, ny)
            + tx * ty * value(nx, ny)
        )
        return self.z_offset + self.height_range * normalized


def _material(parent, ambient, diffuse):
    material = ET.SubElement(parent, 'material')
    ET.SubElement(material, 'ambient').text = ambient
    ET.SubElement(material, 'diffuse').text = diffuse


def _dome_model(name, x, y, radius, terrain_z):
    model = ET.Element('model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    # Placing the sphere centre on the surface leaves one hemisphere visible.
    ET.SubElement(model, 'pose').text = f'{x:.4f} {y:.4f} {terrain_z:.4f} 0 0 0'
    link = ET.SubElement(model, 'link', name='dome')
    for tag in ('collision', 'visual'):
        shape = ET.SubElement(link, tag, name=tag)
        geometry = ET.SubElement(shape, 'geometry')
        sphere = ET.SubElement(geometry, 'sphere')
        ET.SubElement(sphere, 'radius').text = f'{radius:.4f}'
        if tag == 'visual':
            _material(shape, '0.50 0.20 0.10 1', '0.65 0.28 0.14 1')
    return model


def _hallway_model(name, start, end, sampler, width=HALLWAY_WIDTH,
                   height=HALLWAY_HEIGHT):
    x1, y1 = start
    x2, y2 = end
    z1, z2 = sampler.height(x1, y1), sampler.height(x2, y2)
    horizontal_length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)
    pitch = -math.atan2(z2 - z1, horizontal_length)
    length = math.hypot(horizontal_length, z2 - z1)

    model = ET.Element('model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    # The corridor follows the endpoint elevations. Its floor is buried 0.5 m
    # so small heightmap variations do not leave a visible gap underneath it.
    center_z = (z1 + z2) / 2.0 + height / 2.0 - 0.5
    ET.SubElement(model, 'pose').text = (
        f'{(x1 + x2) / 2.0:.4f} {(y1 + y2) / 2.0:.4f} {center_z:.4f} '
        f'0 {pitch:.6f} {yaw:.6f}')
    link = ET.SubElement(model, 'link', name='hallway')
    for tag in ('collision', 'visual'):
        shape = ET.SubElement(link, tag, name=tag)
        geometry = ET.SubElement(shape, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = f'{length:.4f} {width:.4f} {height:.4f}'
        if tag == 'visual':
            _material(shape, '0.24 0.25 0.27 1', '0.38 0.40 0.43 1')
    return model


def generate_base_models(worlds_dir, world, *, cluster_count=NUM_BASE_CLUSTERS,
                         seed=None):
    """Append randomly positioned Mars-base clusters and return their layout."""
    if cluster_count < 0:
        raise ValueError('cluster_count cannot be negative')
    terrain = world.find("model[@name='usgs_utah_terrain']")
    heightmap = terrain.find(".//visual[@name='terrain_visual']/geometry/heightmap")
    sampler = TerrainSampler(worlds_dir, heightmap)
    rng = random.Random(seed)
    layouts = []
    cluster_centres = []

    for cluster_index in range(cluster_count):
        for _ in range(1000):
            if cluster_index < min(NEAR_ORIGIN_CLUSTERS, cluster_count):
                angle = rng.uniform(0.0, 2.0 * math.pi)
                distance = rng.uniform(NEAR_ORIGIN_MIN_DISTANCE,
                                       NEAR_ORIGIN_MAX_DISTANCE)
                cx = distance * math.cos(angle)
                cy = distance * math.sin(angle)
            else:
                cx = rng.uniform(-850.0, 850.0)
                cy = rng.uniform(-850.0, 850.0)
                if math.hypot(cx, cy) < NEAR_ORIGIN_MAX_DISTANCE:
                    continue
            minimum_spacing = (NEAR_CLUSTER_SPACING
                               if cluster_index < NEAR_ORIGIN_CLUSTERS
                               else FAR_CLUSTER_SPACING)
            if any(math.hypot(cx - ox, cy - oy) < minimum_spacing
                   for ox, oy in cluster_centres):
                continue
            break
        else:
            raise RuntimeError('Unable to place terrain base clusters')
        cluster_centres.append((cx, cy))

        dome_count = rng.randint(MIN_DOMES_PER_CLUSTER, MAX_DOMES_PER_CLUSTER)
        domes = [(cx, cy, rng.uniform(12.0, 18.0))]
        angle_offset = rng.uniform(0.0, 2.0 * math.pi)
        for dome_index in range(1, dome_count):
            angle = angle_offset + 2.0 * math.pi * (dome_index - 1) / (dome_count - 1)
            distance = rng.uniform(38.0, 65.0)
            radius = rng.uniform(8.0, 14.0)
            domes.append((cx + distance * math.cos(angle),
                          cy + distance * math.sin(angle), radius))

        for dome_index, (x, y, radius) in enumerate(domes):
            world.append(_dome_model(
                f'mars_base_{cluster_index + 1}_dome_{dome_index + 1}',
                x, y, radius, sampler.height(x, y)))
        # Build a connected, branching layout. Every new dome connects to one
        # existing dome, so some domes naturally become junctions instead of
        # every corridor meeting only at the centre.
        connections = []
        for dome_index in range(1, len(domes)):
            connections.append((rng.randrange(dome_index), dome_index))

        for hallway_index, (first, second) in enumerate(connections):
            world.append(_hallway_model(
                f'mars_base_{cluster_index + 1}_hallway_{hallway_index + 1}',
                domes[first][:2], domes[second][:2], sampler))

        layouts.append({'centre': (cx, cy), 'domes': domes,
                        'connections': connections})
    return layouts
