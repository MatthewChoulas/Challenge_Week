import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty


CONTROL_FREQ = 20.0

VELOCITY_KP = 8.0
HEADING_KP = 2.0
MAX_ANGULAR_SPEED = 0.6

LOOKAHEAD_DISTANCE = 0.40

PATH_END_TOLERANCE = 0.1
OBSTACLE_STOP_DISTANCE = 5.0
ROVER_CORRIDOR_HALF_WIDTH = 1.0
REPLAN_REQUEST_PERIOD = 1.0


def wrap_angle(angle):
    """Wrap an angle to the shortest signed rotation in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PathController(Node):

    def __init__(self):
        super().__init__('path_controller')

        self.path = []
        self.path_progress = 0.0
        self.have_odom = False
        self.motion_enabled = True
        self.max_speed = self.declare_parameter('max_speed', 100.0).value
        self.obstacle_stop_distance = float(self.declare_parameter(
            'obstacle_stop_distance', OBSTACLE_STOP_DISTANCE).value)
        self.corridor_half_width = float(self.declare_parameter(
            'obstacle_corridor_half_width', ROVER_CORRIDOR_HALF_WIDTH).value)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.obstacle_points = np.empty((0, 2), dtype=np.float32)
        self.last_replan_request_ns = 0

        self.path_subscriber = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            1
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/model/robot/odometry',
            self.odom_callback,
            1
        )

        retained = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.obstacle_subscriber = self.create_subscription(
            OccupancyGrid,
            '/local_occupancy_map',
            self.obstacle_callback,
            retained
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/model/robot/cmd_vel',
            10
        )
        self.replan_publisher = self.create_publisher(
            Empty,
            '/replan_path',
            10
        )
        self.motion_subscription = self.create_subscription(
            Bool, '/mission_motion_enabled', self.motion_callback, retained)

        self.control_timer = self.create_timer(
            1.0 / CONTROL_FREQ,
            self.control_callback
        )

        self.get_logger().info('Holonomic path controller started')

    def path_callback(self, msg):
        self.path = list(msg.poses)
        # A replanned path has different segment indices. Reproject onto it.
        self.path_progress = 0.0

    def motion_callback(self, msg):
        self.motion_enabled = bool(msg.data)
        if not self.motion_enabled:
            self.stop_robot()

    def odom_callback(self, msg):
        self.have_odom = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)

        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def obstacle_callback(self, msg):
        expected = msg.info.width * msg.info.height
        if msg.info.resolution <= 0 or len(msg.data) != expected:
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        rows, columns = np.where(grid >= 100)
        self.obstacle_points = np.column_stack((
            msg.info.origin.position.x + (columns + 0.5) * msg.info.resolution,
            msg.info.origin.position.y + (rows + 0.5) * msg.info.resolution,
        )).astype(np.float32)

    def stop_robot(self):
        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)

    def obstacle_blocks_motion(self, direction_x, direction_y):
        """Check the short corridor directly ahead of the requested motion."""
        if len(self.obstacle_points) == 0:
            return False
        length = math.hypot(direction_x, direction_y)
        if length < 1e-9:
            return False
        unit_x, unit_y = direction_x / length, direction_y / length
        offsets = self.obstacle_points - (self.robot_x, self.robot_y)
        forward = offsets[:, 0] * unit_x + offsets[:, 1] * unit_y
        sideways = np.abs(offsets[:, 0] * unit_y - offsets[:, 1] * unit_x)
        return bool(np.any(
            (forward >= 0.0) & (forward <= self.obstacle_stop_distance)
            & (sideways <= self.corridor_half_width)))

    def request_replan(self):
        now = self.get_clock().now().nanoseconds
        if (now - self.last_replan_request_ns
                < int(REPLAN_REQUEST_PERIOD * 1e9)):
            return
        self.replan_publisher.publish(Empty())
        self.last_replan_request_ns = now
        self.get_logger().warning(
            'Obstacle in stopping corridor; stopped and requested A* replan',
            throttle_duration_sec=2.0)

    def select_target(self):
        if len(self.path) == 0:
            return None

        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in self.path]
        if len(points) == 1:
            return points[0]

        # Project onto the closest remaining segment. Progress is a segment
        # index plus its fractional position, and never decreases on this path.
        best_distance = math.inf
        progress = self.path_progress
        for index in range(min(int(progress), len(points) - 2), len(points) - 1):
            ax, ay = points[index]
            bx, by = points[index + 1]
            dx, dy = bx - ax, by - ay
            length_squared = dx * dx + dy * dy
            fraction = 0.0 if length_squared == 0 else (
                ((self.robot_x - ax) * dx + (self.robot_y - ay) * dy)
                / length_squared
            )
            fraction = max(max(0.0, progress - index), min(1.0, fraction))
            px, py = ax + fraction * dx, ay + fraction * dy
            distance = math.hypot(px - self.robot_x, py - self.robot_y)
            if distance < best_distance:
                best_distance = distance
                self.path_progress = index + fraction

        # Walk forward by arc length, interpolating instead of jumping between
        # grid-cell centres or selecting distant points behind the robot.
        remaining = LOOKAHEAD_DISTANCE
        first_segment = min(int(self.path_progress), len(points) - 2)
        for index in range(first_segment, len(points) - 1):
            fraction = max(0.0, self.path_progress - index)
            ax, ay = points[index]
            bx, by = points[index + 1]
            px = ax + fraction * (bx - ax)
            py = ay + fraction * (by - ay)
            length = math.hypot(bx - px, by - py)
            if length > 0 and remaining <= length:
                ratio = remaining / length
                return px + ratio * (bx - px), py + ratio * (by - py)
            remaining -= length

        return points[-1]

    def control_callback(self):
        if (not self.motion_enabled or not self.have_odom or
                len(self.path) == 0):
            self.stop_robot()
            return

        final_pose = self.path[-1].pose

        distance_to_final_pos = math.hypot(
            final_pose.position.x - self.robot_x,
            final_pose.position.y - self.robot_y)

        if distance_to_final_pos <= PATH_END_TOLERANCE:
            self.stop_robot()
            return

        target = self.select_target()
        target_x, target_y = target

        # error in odom frame
        dx_odom = target_x - self.robot_x
        dy_odom = target_y - self.robot_y

        # Transform error into robot frame
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        dx_robot = cos_yaw * dx_odom + sin_yaw * dy_odom
        dy_robot = -sin_yaw * dx_odom + cos_yaw * dy_odom

        distance = math.hypot(dx_robot, dy_robot)

        if distance < 1e-9:
            self.stop_robot()
            return

        speed = min(VELOCITY_KP * distance, self.max_speed)
        desired_yaw = math.atan2(dy_odom, dx_odom)
        heading_error = wrap_angle(desired_yaw - self.robot_yaw)

        if self.obstacle_blocks_motion(dx_odom, dy_odom):
            self.stop_robot()
            self.request_replan()
            return

        cmd = Twist()

        cmd.linear.x = speed * (dx_robot / distance)
        cmd.linear.y = speed * (dy_robot / distance)
        cmd.angular.z = max(
            -MAX_ANGULAR_SPEED,
            min(MAX_ANGULAR_SPEED, HEADING_KP * heading_error))

        self.cmd_vel_publisher.publish(cmd)


def main():
    rclpy.init()
    node = PathController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.stop_robot()

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
