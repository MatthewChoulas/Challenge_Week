import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


CONTROL_FREQ = 20.0

VELOCITY_KP = 2.0

LOOKAHEAD_DISTANCE = 0.40

PATH_END_TOLERANCE = 0.1

class PathController(Node):

    def __init__(self):
        super().__init__('path_controller')

        self.path = []
        self.path_progress = 0.0
        self.have_odom = False
        self.max_speed = self.declare_parameter("max_speed", 100.0).value

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

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

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/model/robot/cmd_vel',
            10
        )

        self.control_timer = self.create_timer(
            1.0 / CONTROL_FREQ,
            self.control_callback
        )

        self.get_logger().info('Holonomic path controller started')
    

    def path_callback(self, msg):
        self.path = list(msg.poses)
        # A replanned path has different segment indices. Reproject onto it.
        self.path_progress = 0.0


    def odom_callback(self, msg):
        self.have_odom = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)

        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)


    def stop_robot(self):
        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)


    def select_target(self):
        if len(self.path) == 0:
            return None

        points = [(pose.pose.position.x, pose.pose.position.y) for pose in self.path]
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
                ((self.robot_x - ax) * dx + (self.robot_y - ay) * dy) / length_squared
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
        for index in range(min(int(self.path_progress), len(points) - 2), len(points) - 1):
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
        if not self.have_odom or len(self.path) == 0:
            self.stop_robot()
            return

        final_pose = self.path[-1].pose

        distance_to_final_pos = math.hypot(final_pose.position.x - self.robot_x, final_pose.position.y - self.robot_y)

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

        cmd = Twist()

        cmd.linear.x = speed * (dx_robot / distance)
        cmd.linear.y = speed * (dy_robot / distance)
        cmd.angular.z = 0.0

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
