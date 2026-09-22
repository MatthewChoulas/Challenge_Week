"""Generate one list of three WGS84 goals within the terrain square."""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from .utm_conversion import local_to_latlon


def generate_waypoints(latitude, longitude, minimum_start, minimum_pair, seed):
    """Sample local metres, enforce spacing, then convert to GPS degrees."""
    if not all(math.isfinite(v) and v >= 0 for v in (minimum_start, minimum_pair)):
        raise ValueError('Minimum distances must be finite and nonnegative')
    rng = random.Random(None if seed < 0 else seed)
    local = []
    gps = []
    for _ in range(10000):
        x, y = rng.uniform(-1000, 1000), rng.uniform(-1000, 1000)
        if math.hypot(x, y) < minimum_start:
            continue
        if any(math.hypot(x - px, y - py) < minimum_pair for px, py in local):
            continue
        lat, lon = local_to_latlon(x, y, latitude, longitude)
        local.append((x, y))
        gps.append((lat, lon))
        if len(gps) == 3:
            return gps
    raise ValueError('Cannot place three waypoints; reduce minimum distances')


class GpsWaypointPublisher(Node):
    def __init__(self):
        super().__init__('gps_waypoint_publisher')
        lat = self.declare_parameter('origin_latitude', 38.42287240335025).value
        lon = self.declare_parameter('origin_longitude', -110.78495572815902).value
        start = self.declare_parameter('min_distance_from_start', 200.0).value
        pair = self.declare_parameter('min_distance_between_goals', 200.0).value
        seed = self.declare_parameter('random_seed', -1).value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Float64MultiArray, '/gps_waypoints', qos)
        self.message = Float64MultiArray()
        self.message.layout.dim = [
            MultiArrayDimension(label='waypoints', size=3, stride=6),
            MultiArrayDimension(label='latitude_longitude_degrees', size=2, stride=2)]
        goals = generate_waypoints(lat, lon, start, pair, seed)
        self.message.data = [value for goal in goals for value in goal]
        for index, (lat, lon) in enumerate(goals):
            self.get_logger().info(f'Goal {index + 1}: latitude={lat:.10f}, longitude={lon:.10f}')
        self.publisher.publish(self.message)
        # Repeat the same list for volatile subscribers; never regenerate it.
        self.timer = self.create_timer(1.0, self.publish_waypoints)

    def publish_waypoints(self):
        self.publisher.publish(self.message)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
