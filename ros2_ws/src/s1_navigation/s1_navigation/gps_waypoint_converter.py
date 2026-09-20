"""Convert GPS goals to local poses and display them in RViz and Gazebo."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from PIL import Image
from pyproj import Geod
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from ros_gz_interfaces.srv import SpawnEntity

MARKER_CLEARANCE = 20.0


def marker_geometry(terrain_z):
    """Return cylinder length and centre with bottom=0 and top=terrain+20."""
    height = terrain_z + MARKER_CLEARANCE
    if not math.isfinite(height) or height <= 0:
        raise ValueError('Terrain elevation must be above -20 m for a base at Z=0')
    return height, height / 2.0


class GpsWaypointConverter(Node):
    def __init__(self):
        super().__init__('gps_waypoint_converter')
        self.latitude = self.declare_parameter('origin_latitude', 38.42287240335025).value
        self.longitude = self.declare_parameter('origin_longitude', -110.78495572815902).value
        self.geod = Geod(ellps='WGS84')
        worlds = Path(get_package_share_directory('s1_navigation')) / 'worlds'
        # Read the displayed terrain asset from the world so marker heights
        # follow its configured resolution, vertical scale and offset.
        heightmap = ET.parse(worlds / 'usgs_utah.sdf').find(
            ".//visual[@name='terrain_visual']/geometry/heightmap")
        self.heightmap = Image.open(worlds / heightmap.findtext('uri'))
        self.size, _, self.height_range = map(float, heightmap.findtext('size').split())
        self.z_offset = float(heightmap.findtext('pos').split()[2])
        self.get_logger().info(
            f'Marker terrain: {heightmap.findtext("uri")} '
            f'({self.heightmap.width} x {self.heightmap.height})')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(
            Float64MultiArray, '/gps_waypoints', self.waypoint_callback, qos)
        self.publisher = self.create_publisher(PoseArray, '/waypoints', qos)
        self.marker_publisher = self.create_publisher(MarkerArray, '/waypoint_markers', qos)
        self.spawn_client = self.create_client(SpawnEntity, '/world/usgs_utah/create')
        self.goals = None
        self.pending = []
        self.spawn_future = None
        self.timer = self.create_timer(1.0, self.spawn_next_marker)

    def terrain_height(self, x, y):
        # USGS raster rows run from north (+Y) to south (-Y).
        px = (x / self.size + 0.5) * (self.heightmap.width - 1)
        py = (0.5 - y / self.size) * (self.heightmap.height - 1)
        ix, iy = math.floor(px), math.floor(py)
        tx, ty = px - ix, py - iy
        def pixel(dx, dy):
            return self.heightmap.getpixel((min(ix + dx, self.heightmap.width - 1),
                                           min(iy + dy, self.heightmap.height - 1))) / 65535.0
        value = ((1-tx)*(1-ty)*pixel(0, 0) + tx*(1-ty)*pixel(1, 0)
                 + (1-tx)*ty*pixel(0, 1) + tx*ty*pixel(1, 1))
        return self.z_offset + self.height_range * value

    def waypoint_callback(self, message):
        # One list per launch, matching the original planner's behaviour.
        if self.goals is not None:
            return
        if len(message.data) != 6 or not all(math.isfinite(v) for v in message.data):
            self.get_logger().error('Expected three latitude/longitude pairs')
            return
        goals = []
        for lat, lon in zip(message.data[::2], message.data[1::2]):
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                return
            azimuth, _, distance = self.geod.inv(self.longitude, self.latitude, lon, lat)
            x = distance * math.sin(math.radians(azimuth))
            y = distance * math.cos(math.radians(azimuth))
            if abs(x) > 1000 or abs(y) > 1000:
                self.get_logger().error('Waypoint outside terrain square')
                return
            goals.append((x, y, self.terrain_height(x, y)))
        self.goals = goals
        poses = PoseArray()
        poses.header.frame_id = 'odom'
        poses.header.stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for index, (x, y, z) in enumerate(goals):
            height, center_z = marker_geometry(z)
            self.get_logger().info(
                f'Waypoint {index + 1}: ground Z={z:.3f}, '
                f'cylinder bottom=0, top={height:.3f} m (20 m clearance)')
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            marker = Marker()
            marker.header = poses.header
            marker.ns, marker.id = 'terrain_waypoints', index
            marker.type, marker.action = Marker.CYLINDER, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = x, y
            # Bottom = centre - height/2 = 0; top = terrain elevation + 20.
            marker.pose.position.z = center_z
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 2.0
            marker.scale.z = height
            marker.color.g = marker.color.a = 1.0
            markers.markers.append(marker)
        self.publisher.publish(poses)
        self.marker_publisher.publish(markers)
        self.pending = list(enumerate(goals))
        self.get_logger().info('Published three local waypoints and RViz markers')

    def spawn_next_marker(self):
        if not self.pending or self.spawn_future is not None:
            return
        if not self.spawn_client.service_is_ready():
            return
        index, (x, y, z) = self.pending[0]
        height, center_z = marker_geometry(z)
        request = SpawnEntity.Request()
        name = f'gps_waypoint_{index + 1}'
        request.entity_factory.name = name
        # SpawnEntity's pose overrides the SDF model pose, including its
        # default (zero) value, so set the factory pose explicitly.
        request.entity_factory.pose.position.x = x
        request.entity_factory.pose.position.y = y
        request.entity_factory.pose.position.z = center_z
        request.entity_factory.pose.orientation.w = 1.0
        request.entity_factory.sdf = f'''<sdf version="1.9"><model name="{name}">
          <static>true</static><pose>{x} {y} {center_z} 0 0 0</pose>
          <link name="marker"><visual name="post"><geometry><cylinder>
          <radius>1</radius><length>{height}</length></cylinder></geometry><material>
          <ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse>
          </material></visual></link></model></sdf>'''
        self.spawn_future = self.spawn_client.call_async(request)
        self.spawn_future.add_done_callback(self.spawn_done)

    def spawn_done(self, future):
        self.spawn_future = None
        try:
            if future.result().success:
                self.get_logger().info(f'Gazebo waypoint {self.pending[0][0] + 1} created')
                self.pending.pop(0)
            else:
                self.get_logger().warning('Gazebo marker rejected; retrying')
        except Exception as error:
            self.get_logger().warning(f'Gazebo marker failed; retrying: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
