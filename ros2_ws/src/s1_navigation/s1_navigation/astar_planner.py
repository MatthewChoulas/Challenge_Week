import math
import heapq

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid, Path

REPLAN_FREQ = 3.0

GOAL_TOLERANCE = 0.3

OBSTACLE_COST = 100

COSTMAP_WEIGHT = 3.0

GRID_WIDTH_CELLS = 250
GRID_LENGTH_CELLS = 250


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')
        self.costmap = None

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.waypoints = []
        self.have_waypoints = False

        self.current_waypoint_index = 0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            qos
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/model/robot/odometry',
            self.odom_callback,
            10
        )

        self.waypoint_subscriber = self.create_subscription(
            PoseArray,
            '/waypoints',
            self.waypoint_callback,
            qos
        )

        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/current_goal',
            10
        )

        self.planning_timer = self.create_timer(
            1.0 / REPLAN_FREQ,
            self.planning_callback
        )

        self.get_logger().info('A* planner started')


    def costmap_callback(self, msg):
        self.costmap = msg


    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y


    def waypoint_callback(self, msg):
        if self.have_waypoints:
            return

        self.waypoints = list(msg.poses)
        self.current_waypoint_index = 0
        self.have_waypoints = True

        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')


    def world_to_grid(self, x, y):
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution

        grid_x = int(math.floor((x - origin_x) / resolution))
        grid_y = int(math.floor((y - origin_y) / resolution))

        return grid_x, grid_y


    def grid_to_world(self, grid_x, grid_y):
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution

        x = (origin_x + (grid_x + 0.5) * resolution)
        y = (origin_y + (grid_y + 0.5) * resolution)

        return x, y
    

    def in_grid_bounds(self, x, y):
        if x < GRID_WIDTH_CELLS and x >= 0 and y < GRID_LENGTH_CELLS and y >= 0:
            return True

        return False


    def current_goal(self):
        if not self.have_waypoints:
            return None

        if self.current_waypoint_index >= len(self.waypoints):
            return None

        return self.waypoints[self.current_waypoint_index]


    def distance_to_goal(self, goal):
        return math.hypot(goal.position.x - self.robot_x, goal.position.y - self.robot_y)


    def publish_current_goal(self, goal):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose = goal

        self.goal_publisher.publish(msg)


    def astar(self, start, goal):
        width = self.costmap.info.width
        height = self.costmap.info.height

        cost_grid = np.array(self.costmap.data, dtype=np.int16).reshape((height, width))

        start_x, start_y = start
        goal_x, goal_y = goal

        # (dx, dy, distance)
        directions = [
            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1,  1, math.sqrt(2.0)),
            ( 1, -1, math.sqrt(2.0)),
            ( 1,  1, math.sqrt(2.0)),
        ]

        # in a star g and h are used to represent the current total cost of the path traveled and the heuristic(i.e. distance to goal) respectively
        # the total cost function f = g + h 

        g_score = np.full((height, width), np.inf, dtype=np.float32)
        g_score[start_y, start_x] = 0.0
        
        # nodes already visited
        closed = np.zeros((height, width), dtype=bool)

        came_from = {}

        # heap of nodes to visit
        open_heap = []

        start_h = math.hypot(goal_x - start_x, goal_y - start_y)
        heapq.heappush(open_heap, (start_h, 0.0, start_x, start_y))

        while open_heap:

            f_score, current_g, x, y = heapq.heappop(open_heap)

            if closed[y, x]:
                continue

            closed[y, x] = True

            # Goal reached
            if x == goal_x and y == goal_y:
                return self.reconstruct_path(came_from, start, goal)

            for dx, dy, distance in directions:

                nx = x + dx
                ny = y + dy

                if not self.in_grid_bounds(nx, ny):
                    continue

                if closed[ny, nx]:
                    continue

                cell_cost = cost_grid[ny, nx]

                if cell_cost >= OBSTACLE_COST:
                    continue

                traversal_cost_multiplier = 1.0 + COSTMAP_WEIGHT * (cell_cost / 100.0)

                step_cost = distance * traversal_cost_multiplier

                new_g = current_g + step_cost

                if new_g >= g_score[ny, nx]:
                    continue

                came_from[(nx, ny)] = (x, y)

                g_score[ny, nx] = new_g

                heuristic = math.hypot(goal_x - nx, goal_y - ny)

                f_score = new_g + heuristic

                heapq.heappush(open_heap,(f_score, new_g, nx, ny))

        return None


    def reconstruct_path(self, came_from, start, goal):
        path = [goal]
        current = goal

        while current != start:
            if current not in came_from:
                return None

            current = came_from[current]

            path.append(current)

        path.reverse()

        return path


    def publish_path(self, grid_path):
        msg = Path()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        if grid_path is not None:
            for grid_x, grid_y in grid_path:
                x, y = self.grid_to_world(grid_x, grid_y)

                pose = PoseStamped()

                pose.header = msg.header

                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.1
                pose.pose.orientation.w = 1.0

                msg.poses.append(pose)

        self.path_publisher.publish(msg)

    def planning_callback(self):
        if self.costmap is None:
            return

        if not self.have_waypoints:
            return

        goal = self.current_goal()

        if goal is None:
            self.publish_path([])
            return

        distance = self.distance_to_goal(goal)

        if distance <= GOAL_TOLERANCE:
            self.current_waypoint_index += 1
            goal = self.current_goal()

            if goal is None:
                self.get_logger().info('All waypoints reached!')
                self.publish_path([])
                return

        self.publish_current_goal(goal)

        start_grid = self.world_to_grid(self.robot_x, self.robot_y)
        goal_grid = self.world_to_grid(goal.position.x, goal.position.y)

        grid_path = self.astar(start_grid, goal_grid)

        if grid_path is None:
            self.publish_path([])
            return

        self.publish_path(grid_path)
    

def main():
    rclpy.init()
    node = AStarPlanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




