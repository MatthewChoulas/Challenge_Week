"""Publish the complete Gazebo terrain as a retained elevation GridMap."""
from array import array
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

MAX_TRAVERSABLE_SLOPE_DEGREES = 30.0


def maximum_filter_3x3(values):
    """Assign each cell the largest value in its 3-by-3 neighborhood."""
    padded = np.pad(values, 1, mode='edge')
    rows, columns = values.shape
    neighborhoods = [
        padded[row_offset:row_offset + rows,
               column_offset:column_offset + columns]
        for row_offset in range(3)
        for column_offset in range(3)
    ]
    return np.maximum.reduce(neighborhoods)


def grid_map_layer(values):
    """Pack an X-by-Y NumPy matrix into GridMap's column-major layout."""
    rows, columns = values.shape
    layer = Float32MultiArray()
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=rows,
                            stride=rows * columns),
        MultiArrayDimension(label='row_index', size=columns, stride=columns)]
    layer.data = array('f', values.astype(np.float32).flatten(order='F'))
    return layer


def build_elevation_grid(worlds, downsample_factor=1):
    """Average terrain cells, retaining the full map extent and world axes."""
    if not isinstance(downsample_factor, int) or downsample_factor < 1:
        raise ValueError('downsample_factor must be a positive integer')
    worlds = Path(worlds)
    heightmap = ET.parse(worlds / 'usgs_utah.sdf').find(
        ".//visual[@name='terrain_visual']/geometry/heightmap")
    size_x, size_y, size_z = map(float, heightmap.findtext('size').split())
    offset_x, offset_y, offset_z = map(float, heightmap.findtext('pos').split())
    with Image.open(worlds / heightmap.findtext('uri')) as image:
        elevations = np.asarray(image, dtype=np.float32) / 65535.0
    # Heightmap samples are vertices. Average each four-vertex cell so the
    # grid covers exactly 2000 x 2000 m with correctly positioned cell centres.
    cells = (elevations[:-1, :-1] + elevations[1:, :-1]
             + elevations[:-1, 1:] + elevations[1:, 1:]) * 0.25
    cells = offset_z + cells * size_z
    if downsample_factor > 1:
        rows, columns = cells.shape
        factor = downsample_factor
        if rows % factor or columns % factor:
            raise ValueError('downsample_factor must divide both grid dimensions')
        # Average blocks instead of dropping samples: no coordinate shift or
        # loss of map coverage, and narrow features contribute to the result.
        cells = cells.reshape(rows // factor, factor,
                              columns // factor, factor).mean(axis=(1, 3))
    rows, columns = cells.shape
    resolution = size_x / columns
    if not np.isclose(resolution, size_y / rows):
        raise ValueError('GridMap requires square cells')

    # np.gradient returns rise/run in image-row (Y) and image-column (X)
    # directions. Their magnitude is the maximum local slope at each cell.
    gradient_y, gradient_x = np.gradient(cells, resolution)
    slope_degrees = np.degrees(np.arctan(np.hypot(gradient_x, gradient_y)))
    traversability_cost = np.clip(
        slope_degrees / MAX_TRAVERSABLE_SLOPE_DEGREES, 0.0, 1.0)
    # Inflate steep terrain by one grid cell in every direction. The planner
    # therefore checks the worst cost underneath an approximately 3x3-cell
    # footprint instead of treating the rover as a point.
    traversability_cost = maximum_filter_3x3(traversability_cost)

    message = GridMap()
    message.header.frame_id = 'odom'
    message.info.resolution = resolution
    message.info.length_x = size_x
    message.info.length_y = size_y
    message.info.pose.position.x = offset_x
    message.info.pose.position.y = offset_y
    message.info.pose.orientation.w = 1.0
    message.layers = ['elevation', 'slope_degrees', 'traversability_cost']
    message.basic_layers = ['elevation']
    # GridMap index (0,0) is the +X,+Y corner. The USGS image starts at the
    # north (+Y) edge and its columns run west-to-east, so only X is reversed
    # when converting image [Y, X] into GridMap [X, Y] indices.
    message.data = [
        grid_map_layer(values[:, ::-1].T)
        for values in (cells, slope_degrees, traversability_cost)
    ]
    return message


class ElevationMapPublisher(Node):
    def __init__(self):
        super().__init__('elevation_map_publisher')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(GridMap, '/elevation_map', qos)
        self.visual_publisher = self.create_publisher(
            GridMap, '/elevation_map_visual', qos)
        factor = self.declare_parameter('visual_downsample_factor', 4).value
        worlds = Path(get_package_share_directory('s1_navigation')) / 'worlds'
        message = build_elevation_grid(worlds)
        visual_message = build_elevation_grid(worlds, factor)
        self.publisher.publish(message)
        self.visual_publisher.publish(visual_message)
        self.get_logger().info(
            f'Published elevation GridMap ({message.info.resolution:.6f} m/cell) on '
            '/elevation_map')
        self.get_logger().info(
            f'Published RViz GridMap ({visual_message.info.resolution:.6f} m/cell) '
            'on /elevation_map_visual; use Transient Local durability in RViz')
        # Stay alive to retain the sample for subscribers that join later.
        # There is no periodic retransmission of this large static map.


def main(args=None):
    rclpy.init(args=args)
    node = ElevationMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
