import xml.etree.ElementTree as ET
import math
import random
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import Pose, PoseArray
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray

NUM_WAYPOINTS = 5
MAX_SAMPLE_ATTEMPTS = 10000

X_MIN = -10.0
X_MAX = 10.0
Y_MIN = -10.0
Y_MAX = 10.0

ROBOT_START_X = 0.0
ROBOT_START_Y = 0.0

MIN_DISTANCE_FROM_START = 2.0

ROBOT_RADIUS = 0.18

GOAL_OBSTACLE_CLEARANCE = 0.5

MIN_DISTANCE_BETWEEN_GOALS = 2.0

RANDOM_SEED = None

PACKAGE_SHARE = Path(get_package_share_directory('s1_navigation'))

WORLD_FILE = PACKAGE_SHARE / 'worlds' / 's1_world.sdf'


def load_world_obstacles(world_file):
    """Read the rock geometry written by obstacle_generator from Gazebo's SDF."""
    world = ET.parse(world_file).getroot().find('world')
    if world is None:
        raise ValueError(f'No world found in {world_file}')
    obstacles = []
    for model in world.findall('model'):
        if not model.get('name', '').startswith('rock_'):
            continue
        pose = [float(value) for value in model.findtext('pose', '').split()]
        radius = float(model.findtext('link/collision/geometry/sphere/radius', 'nan'))
        if (len(pose) != 6 or not all(math.isfinite(value) for value in pose)
                or not math.isfinite(radius) or radius <= 0):
            raise ValueError(f'Invalid generated obstacle: {model.get("name")}')
        obstacles.append({'x': pose[0], 'y': pose[1], 'radius': radius})
    return obstacles


class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')

        self.rng = random.Random(RANDOM_SEED)

        self.obstacles = self.load_obstacles()

        self.get_logger().info(f'Loaded {len(self.obstacles)} obstacles')

        waypoint_qos = QoSProfile(
            depth=1, 
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.waypoint_publisher = self.create_publisher(
            PoseArray,
            '/waypoints',
            waypoint_qos
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            waypoint_qos
        )

        self.waypoints = self.generate_waypoints()

        self.publish_waypoints()

        self.get_logger().info(f'Published {len(self.waypoints)} waypoints on /waypoints')

        self.publish_markers()

        self.get_logger().info(f'Published {len(self.waypoints)} markers on /waypoint_markers')

    def load_obstacles(self):
        world_file = self.declare_parameter('world_file', str(WORLD_FILE)).value
        return load_world_obstacles(world_file)


    def too_close_to_start(self, x, y):
        distance = math.hypot(x - ROBOT_START_X, y - ROBOT_START_Y)
        return distance < MIN_DISTANCE_FROM_START


    def too_close_to_obstacle(self, x, y):
        for obstacle in self.obstacles:
            obstacle_x = obstacle["x"]
            obstacle_y = obstacle["y"]
            obstacle_radius = obstacle["radius"]

            distance = math.hypot(x - obstacle_x,y - obstacle_y)
            minimum_distance = (obstacle_radius + ROBOT_RADIUS + GOAL_OBSTACLE_CLEARANCE)

            if distance < minimum_distance:
                return True

        return False


    def too_close_to_waypoint(self, x, y, waypoints):
        for waypoint_x, waypoint_y in waypoints:
            distance = math.hypot(x - waypoint_x, y - waypoint_y)

            if distance < MIN_DISTANCE_BETWEEN_GOALS:
                return True

        return False


    def is_valid_waypoint(self, x, y, waypoints):
        if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
            return False

        if self.too_close_to_start(x, y):
            return False

        if self.too_close_to_obstacle(x, y):
            return False

        if self.too_close_to_waypoint(x, y, waypoints):
            return False

        return True


    def generate_waypoints(self):
        waypoints = []

        for index in range(NUM_WAYPOINTS):
            for _ in range(MAX_SAMPLE_ATTEMPTS):
                x = self.rng.uniform(X_MIN, X_MAX)
                y = self.rng.uniform(Y_MIN, Y_MAX)

                if not self.is_valid_waypoint(x, y, waypoints):
                    continue

                waypoints.append((x, y))

                self.get_logger().info(
                    f'Waypoint {index + 1}: '
                    f'x={x:.2f}, y={y:.2f} '
                )

                break
            else:
                raise RuntimeError(
                    f"Unable to place waypoint {index + 1} after "
                    f"{MAX_SAMPLE_ATTEMPTS} attempts; check obstacle density and clearance"
                )

        return waypoints


    def publish_waypoints(self):
        msg = PoseArray()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        for x, y in self.waypoints:
            pose = Pose()

            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)

    def publish_markers(self):
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(self.waypoints):
            sphere = Marker()

            sphere.header.frame_id = "odom"
            sphere.header.stamp = stamp

            sphere.ns = 'waypoint_spheres'
            sphere.id = i

            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.20

            sphere.pose.orientation.x = 0.0
            sphere.pose.orientation.y = 0.0
            sphere.pose.orientation.z = 0.0
            sphere.pose.orientation.w = 1.0

            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4

            sphere.color.r = 0.1
            sphere.color.g = 1.0
            sphere.color.b = 0.1
            sphere.color.a = 1.0

            msg.markers.append(sphere)

            label = Marker()

            label.header.frame_id = "odom"
            label.header.stamp = stamp

            label.ns = 'waypoint_labels'
            label.id = i

            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.7

            label.pose.orientation.x = 0.0
            label.pose.orientation.y = 0.0
            label.pose.orientation.z = 0.0
            label.pose.orientation.w = 1.0

            label.scale.z = 0.4

            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0

            label.text = f'Goal{i + 1}'

            msg.markers.append(label)

        self.marker_publisher.publish(msg)



def main():
    rclpy.init()
    node = WaypointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
