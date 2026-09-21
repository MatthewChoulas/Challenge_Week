"""Build a rolling 3D obstacle map while rejecting returns on the DEM surface."""
from collections import deque
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy)
from s1_navigation.terrain_bases import TerrainSampler
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

LOCAL_MAP_SIZE = 50.0
VOXEL_SIZE = 0.25
GLOBAL_MAP_RESOLUTION = 1.0
GLOBAL_CLEARANCE_DISTANCE = 6.0
TERRAIN_CLEARANCE = 1.0
VOXEL_LIFETIME = 3.0
FREE_SPACE_UPDATE = -0.25
OCCUPIED_SPACE_UPDATE = 0.7
MIN_LOG_ODDS = -5.0
MAX_LOG_ODDS = 5.0
FREE_PROBABILITY = 0.35
OCCUPIED_PROBABILITY = 0.70
MIN_HIT_OBSERVATIONS = 2
RAY_CLEARING_STRIDE = 1
ODOM_HISTORY_SECONDS = 5.0
MAX_ODOM_CLOUD_TIME_DIFFERENCE = 0.20
GOAL_POST_RADIUS = 1.0
GOAL_POST_FILTER_MARGIN = 0.25


def quaternion_rotation_matrix(quaternion):
    """Return the 3-by-3 rotation matrix for a ROS quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0:
        return np.eye(3)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def probability_from_log_odds(log_odds):
    return 1.0 / (1.0 + math.exp(-log_odds))


def is_goal_post_return(x, y, goal_posts):
    """Return true when a lidar return lies on a visual-only goal post."""
    if len(goal_posts) == 0:
        return False
    radius = GOAL_POST_RADIUS + GOAL_POST_FILTER_MARGIN
    offsets = goal_posts - (x, y)
    return bool(np.any(np.sum(offsets * offsets, axis=1) <= radius * radius))


def normalize_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)
    return quaternion / norm if norm else np.array([0.0, 0.0, 0.0, 1.0])


def interpolate_quaternion(first, second, ratio):
    """Spherically interpolate ROS XYZW quaternions."""
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second, dot = -second, -dot
    if dot > 0.9995:
        return normalize_quaternion(first + ratio * (second - first))
    angle = math.acos(max(-1.0, min(1.0, dot)))
    scale = math.sin(angle)
    return ((math.sin((1.0 - ratio) * angle) / scale) * first
            + (math.sin(ratio * angle) / scale) * second)


def interpolate_odom_pose(history, stamp_ns):
    """Interpolate an odometry history at a lidar cloud timestamp."""
    if not history:
        return None
    if stamp_ns <= history[0][0]:
        sample = history[0]
        return (sample[1], sample[2]) if history[0][0] - stamp_ns <= int(
            MAX_ODOM_CLOUD_TIME_DIFFERENCE * 1e9) else None
    if stamp_ns >= history[-1][0]:
        sample = history[-1]
        return (sample[1], sample[2]) if stamp_ns - history[-1][0] <= int(
            MAX_ODOM_CLOUD_TIME_DIFFERENCE * 1e9) else None
    for earlier, later in zip(history, list(history)[1:]):
        if earlier[0] <= stamp_ns <= later[0]:
            ratio = (stamp_ns - earlier[0]) / (later[0] - earlier[0])
            position = earlier[1] + ratio * (later[1] - earlier[1])
            return position, interpolate_quaternion(earlier[2], later[2], ratio)
    return None


def ray_voxel_keys(start, end, voxel_size):
    """Return intersected voxels using fast 3D digital differential analysis."""
    sx, sy, sz = start
    ex, ey, ez = end
    ix, iy, iz = (math.floor(sx / voxel_size), math.floor(sy / voxel_size),
                  math.floor(sz / voxel_size))
    end_x, end_y, end_z = (math.floor(ex / voxel_size),
                           math.floor(ey / voxel_size),
                           math.floor(ez / voxel_size))
    dx, dy, dz = ex - sx, ey - sy, ez - sz

    def axis_parameters(origin, direction, index):
        if direction > 0.0:
            return 1, ((index + 1) * voxel_size - origin) / direction, voxel_size / direction
        if direction < 0.0:
            return -1, (index * voxel_size - origin) / direction, -voxel_size / direction
        return 0, math.inf, math.inf

    step_x, max_x, delta_x = axis_parameters(sx, dx, ix)
    step_y, max_y, delta_y = axis_parameters(sy, dy, iy)
    step_z, max_z, delta_z = axis_parameters(sz, dz, iz)
    keys = [(ix, iy, iz)]
    while (ix, iy, iz) != (end_x, end_y, end_z):
        if max_x <= max_y and max_x <= max_z:
            ix += step_x
            max_x += delta_x
        elif max_y <= max_z:
            iy += step_y
            max_y += delta_y
        else:
            iz += step_z
            max_z += delta_z
        keys.append((ix, iy, iz))
    return keys


class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')
        self.local_size = float(self.declare_parameter('local_map_size', LOCAL_MAP_SIZE).value)
        self.voxel_size = float(self.declare_parameter('voxel_size', VOXEL_SIZE).value)
        self.terrain_clearance = float(
            self.declare_parameter('terrain_clearance', TERRAIN_CLEARANCE).value)
        self.voxel_lifetime = float(
            self.declare_parameter('voxel_lifetime', VOXEL_LIFETIME).value)
        self.robot_position = np.zeros(3)
        self.robot_rotation = np.eye(3)
        self.have_odom = False
        self.voxels = {}
        self.goal_posts = np.empty((0, 2), dtype=np.float32)
        self.global_occupied_cells = set()
        self.global_map_dirty = True
        self.last_global_publish_ns = 0
        self.odom_history = deque()
        self.scan_count = 0

        worlds = Path(get_package_share_directory('s1_navigation')) / 'worlds'
        heightmap = ET.parse(worlds / 'usgs_utah.sdf').find(
            ".//visual[@name='terrain_visual']/geometry/heightmap")
        self.terrain = TerrainSampler(worlds, heightmap)

        retained_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_subscriber = self.create_subscription(
            PointCloud2, '/scan/points', self.cloud_callback, qos_profile_sensor_data)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/model/robot/odometry', self.odom_callback, 10)
        self.waypoint_subscriber = self.create_subscription(
            PoseArray, '/waypoints', self.waypoint_callback, retained_qos)
        self.cloud_publisher = self.create_publisher(
            PointCloud2, '/local_obstacle_cloud', retained_qos)
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/local_occupancy_map', retained_qos)
        self.global_map_publisher = self.create_publisher(
            OccupancyGrid, '/global_occupancy_map', retained_qos)
        self.global_resolution = float(self.declare_parameter(
            'global_map_resolution', GLOBAL_MAP_RESOLUTION).value)
        self.global_clearance = float(self.declare_parameter(
            'global_clearance_distance', GLOBAL_CLEARANCE_DISTANCE).value)
        self.global_width = int(math.ceil(
            self.terrain.size / self.global_resolution))
        self.global_origin = -self.terrain.size / 2.0
        self.global_grid = np.full(
            (self.global_width, self.global_width), -1, dtype=np.int8)
        self.publish_timer = self.create_timer(0.2, self.publish_maps)
        self.get_logger().info(
            f'Rolling 3D lidar mapper started: {self.local_size:.1f} m region, '
            f'{self.voxel_size:.2f} m voxels, terrain clearance '
            f'{self.terrain_clearance:.2f} m')

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.robot_position = np.array([position.x, position.y, position.z])
        quaternion = normalize_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w))
        self.robot_rotation = quaternion_rotation_matrix(quaternion)
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        self.odom_history.append((stamp_ns, self.robot_position.copy(), quaternion))
        oldest = stamp_ns - int(ODOM_HISTORY_SECONDS * 1e9)
        while self.odom_history and self.odom_history[0][0] < oldest:
            self.odom_history.popleft()
        self.have_odom = True

    def waypoint_callback(self, message):
        """Record visual goal-post centres so lidar mapping can ignore them."""
        self.goal_posts = np.asarray(
            [(pose.position.x, pose.position.y) for pose in message.poses],
            dtype=np.float32).reshape(-1, 2)

    def cloud_callback(self, message):
        if not self.have_odom or message.width * message.height == 0:
            return
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        pose = interpolate_odom_pose(self.odom_history, stamp_ns)
        if pose is None:
            self.get_logger().debug('Waiting for odometry matching lidar timestamp')
            return
        position, quaternion = pose
        rotation = quaternion_rotation_matrix(quaternion)
        points = point_cloud2.read_points_numpy(
            message, field_names=['x', 'y', 'z'], skip_nans=True)
        if points.size == 0:
            return
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)

        # The lidar is 0.30 m above base_link. Rotate that offset and every
        # sensor-frame return using the rover's full roll, pitch, and yaw.
        sensor_origin = position + rotation @ np.array([0.0, 0.0, 0.30])
        world_points = points @ rotation.T + sensor_origin
        half = self.local_size / 2.0
        local = ((np.abs(world_points[:, 0] - position[0]) <= half)
                 & (np.abs(world_points[:, 1] - position[1]) <= half))
        world_points = world_points[local]

        accepted = 0
        observation_time = self.get_clock().now().nanoseconds
        self.scan_count += 1
        for point_index, (x, y, z) in enumerate(world_points):
            if is_goal_post_return(x, y, self.goal_posts):
                continue
            # Ground returns, the thin Gazebo path visual, and small DEM/sensor
            # discrepancies are discarded. The generated base walls and domes
            # extend several metres above this band and remain detectable.
            obstacle_hit = z > self.terrain.height(x, y) + self.terrain_clearance
            keys = None
            if point_index % RAY_CLEARING_STRIDE == 0:
                keys = ray_voxel_keys(sensor_origin, (x, y, z), self.voxel_size)
                free_keys = keys[:-1] if obstacle_hit else keys
                for key in free_keys:
                    self.update_voxel(key, FREE_SPACE_UPDATE, observation_time)
            if obstacle_hit:
                key = (keys[-1] if keys else
                       tuple(np.floor(np.array([x, y, z]) /
                                      self.voxel_size).astype(int)))
                self.update_voxel(key, OCCUPIED_SPACE_UPDATE, observation_time,
                                  hit_scan=self.scan_count)
                accepted += 1
        self.prune_voxels()
        self.get_logger().debug(
            f'Accepted {accepted}/{len(world_points)} local non-terrain returns')

    def update_voxel(self, key, change, observation_time, hit_scan=None):
        old_odds, _, hit_count, last_hit_scan = self.voxels.get(
            key, (0.0, observation_time, 0, -1))
        new_odds = min(MAX_LOG_ODDS, max(MIN_LOG_ODDS, old_odds + change))
        if hit_scan is not None and hit_scan != last_hit_scan:
            hit_count += 1
            last_hit_scan = hit_scan
        elif hit_scan is None and probability_from_log_odds(new_odds) <= FREE_PROBABILITY:
            # A confidently free observation starts a new persistence period.
            hit_count, last_hit_scan = 0, -1
        self.voxels[key] = (new_odds, observation_time, hit_count, last_hit_scan)

    @staticmethod
    def confirmed_occupied(state):
        return (state[2] >= MIN_HIT_OBSERVATIONS
                and probability_from_log_odds(state[0]) >= OCCUPIED_PROBABILITY)

    def voxel_center(self, key):
        return tuple((np.asarray(key, dtype=float) + 0.5) * self.voxel_size)

    def prune_voxels(self):
        half = self.local_size / 2.0
        x, y = self.robot_position[:2]
        oldest = (self.get_clock().now().nanoseconds
                  - int(self.voxel_lifetime * 1e9))
        kept = {}
        for key, state in self.voxels.items():
            vx, vy, _ = self.voxel_center(key)
            if (abs(vx - x) <= half and abs(vy - y) <= half
                    and state[1] >= oldest):
                kept[key] = state
        self.voxels = kept

    def publish_maps(self):
        if not self.have_odom:
            return
        self.prune_voxels()
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='odom')
        occupied_keys = [
            key for key, state in self.voxels.items()
            if self.confirmed_occupied(state)]
        points = [self.voxel_center(key) for key in occupied_keys]
        self.cloud_publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
        for x, y, _ in points:
            gx = int(math.floor((x - self.global_origin) /
                                self.global_resolution))
            gy = int(math.floor((y - self.global_origin) /
                                self.global_resolution))
            if (0 <= gx < self.global_width and 0 <= gy < self.global_width
                    and (gx, gy) not in self.global_occupied_cells):
                self.global_occupied_cells.add((gx, gy))
                self.add_global_obstacle(gx, gy)
                self.global_map_dirty = True

        width = int(math.ceil(self.local_size / self.voxel_size))
        origin_x = self.robot_position[0] - self.local_size / 2.0
        origin_y = self.robot_position[1] - self.local_size / 2.0
        grid = np.full((width, width), -1, dtype=np.int8)
        occupied_cells = set()
        free_cells = set()
        for key, state in self.voxels.items():
            x, y, _ = self.voxel_center(key)
            gx = int((x - origin_x) / self.voxel_size)
            gy = int((y - origin_y) / self.voxel_size)
            if 0 <= gx < width and 0 <= gy < width:
                probability = probability_from_log_odds(state[0])
                if self.confirmed_occupied(state):
                    occupied_cells.add((gx, gy))
                elif probability <= FREE_PROBABILITY:
                    free_cells.add((gx, gy))
        for gx, gy in free_cells - occupied_cells:
            grid[gy, gx] = 0
        for gx, gy in occupied_cells:
            grid[gy, gx] = 100
        message = OccupancyGrid()
        message.header = header
        message.info.resolution = self.voxel_size
        message.info.width = message.info.height = width
        message.info.origin.position.x = origin_x
        message.info.origin.position.y = origin_y
        message.info.origin.orientation.w = 1.0
        message.data = grid.ravel().tolist()
        self.map_publisher.publish(message)

        now_ns = self.get_clock().now().nanoseconds
        if (self.global_map_dirty
                and now_ns - self.last_global_publish_ns >= 1_000_000_000):
            self.publish_global_map(header)
            self.global_map_dirty = False
            self.last_global_publish_ns = now_ns

    def add_global_obstacle(self, grid_x, grid_y):
        """Add one occupied cell and its graded clearance cost permanently."""
        radius = max(1, int(math.ceil(
            self.global_clearance / self.global_resolution)))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                distance = math.hypot(dx, dy) * self.global_resolution
                if distance > self.global_clearance:
                    continue
                x, y = grid_x + dx, grid_y + dy
                if 0 <= x < self.global_width and 0 <= y < self.global_width:
                    value = (100 if distance == 0.0 else
                             max(1, int(round(
                                 99.0 * (1.0 - distance /
                                         self.global_clearance)))))
                    self.global_grid[y, x] = max(
                        int(self.global_grid[y, x]), value)

    def publish_global_map(self, header):
        """Publish the persistent full-terrain obstacle and clearance grid."""
        message = OccupancyGrid()
        message.header = header
        message.info.resolution = self.global_resolution
        message.info.width = message.info.height = self.global_width
        message.info.origin.position.x = self.global_origin
        message.info.origin.position.y = self.global_origin
        message.info.origin.orientation.w = 1.0
        message.data = self.global_grid.ravel().tolist()
        self.global_map_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
