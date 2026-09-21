"""Plan global paths through the terrain traversability GridMap."""

import heapq
import math
import time

from geometry_msgs.msg import PoseArray, PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


REPLAN_FREQ = 1.0
# A* targets a terrain-cell centre; accept a waypoint once the rover is within
# 2 m so it can progress reliably despite grid and odometry quantization.
GOAL_TOLERANCE = 2.0
MAX_TRAVERSABILITY_COST = 1.0
TERRAIN_COST_WEIGHT = 3.0
OBSTACLE_COST_WEIGHT = 8.0


def unpack_grid_map_layer(message, layer_name):
    """Return a layer as conventional array[y increasing, x increasing]."""
    try:
        layer = message.data[message.layers.index(layer_name)]
    except (ValueError, IndexError):
        raise ValueError(f'GridMap has no {layer_name!r} layer') from None
    if len(layer.layout.dim) != 2:
        raise ValueError(f'{layer_name} must have two dimensions')

    # The publisher uses GridMap's standard column-major [X, Y] matrix. Its
    # first indices represent +X,+Y, while NumPy planning arrays start at
    # -X,-Y and are indexed [Y, X].
    rows = layer.layout.dim[1].size
    columns = layer.layout.dim[0].size
    matrix = np.asarray(layer.data, dtype=np.float32).reshape(
        (rows, columns), order='F')
    return matrix[::-1, ::-1].T.copy()


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')
        self.terrain_cost = None
        self.elevation = None
        self.obstacle_cost = None
        self.obstacle_message = None
        self.resolution = 0.0
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.have_odom = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.waypoints = []
        self.have_waypoints = False
        self.current_waypoint_index = 0
        self.last_plan_position = None
        self.last_planned_goal_index = None
        self.replan_requested = False
        self.active_path = []

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_map_subscriber = self.create_subscription(
            GridMap, '/elevation_map', self.grid_map_callback, qos)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/model/robot/odometry', self.odom_callback, 10)
        self.waypoint_subscriber = self.create_subscription(
            PoseArray, '/waypoints', self.waypoint_callback, qos)
        self.obstacle_subscriber = self.create_subscription(
            OccupancyGrid, '/global_occupancy_map',
            self.obstacle_map_callback, qos)
        self.replan_subscriber = self.create_subscription(
            Empty, '/replan_path', self.replan_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/current_goal', 10)
        self.planning_timer = self.create_timer(
            1.0 / REPLAN_FREQ, self.planning_callback)
        self.get_logger().info(
            'A* planner waiting for /elevation_map traversability_cost')

    def grid_map_callback(self, message):
        try:
            cost = unpack_grid_map_layer(message, 'traversability_cost')
            elevation = unpack_grid_map_layer(message, 'elevation')
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        if cost.shape != elevation.shape:
            self.get_logger().error('Terrain cost and elevation shapes differ')
            return
        if message.info.resolution <= 0:
            self.get_logger().error('GridMap resolution must be positive')
            return

        self.terrain_cost = cost
        self.elevation = elevation
        self.height, self.width = cost.shape
        self.resolution = message.info.resolution
        self.origin_x = (message.info.pose.position.x
                         - message.info.length_x / 2.0)
        self.origin_y = (message.info.pose.position.y
                         - message.info.length_y / 2.0)
        self.last_plan_position = None
        self.align_obstacle_map()
        self.get_logger().info(
            f'Received {self.width} x {self.height} terrain grid at '
            f'{self.resolution:.3f} m/cell')

    def obstacle_map_callback(self, message):
        expected = message.info.width * message.info.height
        if message.info.resolution <= 0 or len(message.data) != expected:
            self.get_logger().warning('Ignoring malformed global occupancy grid')
            return
        self.obstacle_message = message
        self.align_obstacle_map()
        # The timer checks whether the remaining route is actually blocked.

    def align_obstacle_map(self):
        """Align the OccupancyGrid obstacle costs with the terrain cells."""
        if self.terrain_cost is None or self.obstacle_message is None:
            return
        message = self.obstacle_message
        source = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width)
        source_origin_x = message.info.origin.position.x
        source_origin_y = message.info.origin.position.y
        if (source.shape == self.terrain_cost.shape
                and math.isclose(message.info.resolution, self.resolution)
                and math.isclose(source_origin_x, self.origin_x)
                and math.isclose(source_origin_y, self.origin_y)):
            aligned = source
        else:
            world_x = self.origin_x + (
                np.arange(self.width) + 0.5) * self.resolution
            world_y = self.origin_y + (
                np.arange(self.height) + 0.5) * self.resolution
            columns = np.floor(
                (world_x - source_origin_x) /
                message.info.resolution).astype(int)
            rows = np.floor(
                (world_y - source_origin_y) /
                message.info.resolution).astype(int)
            valid_x = (columns >= 0) & (columns < message.info.width)
            valid_y = (rows >= 0) & (rows < message.info.height)
            aligned = np.full((self.height, self.width), -1, dtype=np.int16)
            aligned[np.ix_(valid_y, valid_x)] = source[
                np.ix_(rows[valid_y], columns[valid_x])]
        self.obstacle_cost = np.maximum(aligned, 0).astype(np.float32) / 100.0

    def replan_callback(self, _message):
        self.replan_requested = True

    def odom_callback(self, message):
        self.have_odom = True
        self.robot_x = message.pose.pose.position.x
        self.robot_y = message.pose.pose.position.y

    def waypoint_callback(self, message):
        if self.have_waypoints:
            return
        self.waypoints = list(message.poses)
        self.current_waypoint_index = 0
        self.have_waypoints = True
        self.last_planned_goal_index = None
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')

    def world_to_grid(self, x, y):
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def grid_to_world(self, grid_x, grid_y):
        return (self.origin_x + (grid_x + 0.5) * self.resolution,
                self.origin_y + (grid_y + 0.5) * self.resolution)

    def in_grid_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def in_world_bounds(self, x, y):
        return (self.origin_x <= x < self.origin_x + self.width * self.resolution
                and self.origin_y <= y < self.origin_y + self.height * self.resolution)

    def current_goal(self):
        if (not self.have_waypoints or
                self.current_waypoint_index >= len(self.waypoints)):
            return None
        return self.waypoints[self.current_waypoint_index]

    def distance_to_goal(self, goal):
        return math.hypot(
            goal.position.x - self.robot_x,
            goal.position.y - self.robot_y)

    def publish_current_goal(self, goal):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.pose = goal
        self.goal_publisher.publish(message)

    def path_cell_free(self, cell):
        x, y = cell
        if not self.in_grid_bounds(x, y):
            return False
        terrain = self.terrain_cost[y, x]
        obstacle = getattr(self, 'obstacle_cost', None)
        return (np.isfinite(terrain) and terrain < MAX_TRAVERSABILITY_COST
                and (obstacle is None or obstacle[y, x] < 1.0))

    def remaining_path_index(self, start):
        return min(range(len(self.active_path)), key=lambda index: (
            (self.active_path[index][0] - start[0]) ** 2
            + (self.active_path[index][1] - start[1]) ** 2))

    def path_section_clear(self, path):
        for index, cell in enumerate(path):
            if not self.path_cell_free(cell):
                return False
            if index:
                x, y = path[index - 1]
                nx, ny = cell
                if x != nx and y != ny:
                    if not self.path_cell_free((x, ny)) or not self.path_cell_free((nx, y)):
                        return False
        return True

    def repair_path(self, start, nearest):
        """Try 20, 40, then 60 m rejoin distances with bounded search windows."""
        remaining = self.active_path[nearest:]
        distances = [0.0]
        for first, second in zip(remaining, remaining[1:]):
            distances.append(distances[-1] + self.resolution * math.hypot(
                second[0] - first[0], second[1] - first[1]))
        tried = set()
        for distance, margin in ((20.0, 10.0), (40.0, 20.0), (60.0, 30.0)):
            index = next((i for i, value in enumerate(distances)
                          if value >= distance), len(remaining) - 1)
            if index in tried:
                continue
            tried.add(index)
            # Preserve only a suffix that is still collision-free.
            if not self.path_section_clear(remaining[index:]):
                continue
            target = remaining[index]
            padding = int(math.ceil(margin / self.resolution))
            bounds = (max(0, min(start[0], target[0]) - padding),
                      max(0, min(start[1], target[1]) - padding),
                      min(self.width - 1, max(start[0], target[0]) + padding),
                      min(self.height - 1, max(start[1], target[1]) + padding))
            segment = self.astar(start, target, bounds=bounds)
            if segment:
                return segment + remaining[index + 1:]
        return None

    def astar(self, start, goal, bounds=None):
        if not self.in_grid_bounds(*start) or not self.in_grid_bounds(*goal):
            return None

        def traversable(x, y):
            cost = self.terrain_cost[y, x]
            obstacle = getattr(self, 'obstacle_cost', None)
            obstacle_free = obstacle is None or obstacle[y, x] < 1.0
            return (np.isfinite(cost) and cost < MAX_TRAVERSABILITY_COST
                    and obstacle_free)

        if not traversable(*start) or not traversable(*goal):
            return None

        if bounds is None:
            bounds = (0, 0, self.width - 1, self.height - 1)
        xmin, ymin, xmax, ymax = bounds
        if not (xmin <= start[0] <= xmax and ymin <= start[1] <= ymax
                and xmin <= goal[0] <= xmax and ymin <= goal[1] <= ymax):
            return None

        directions = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]
        g_score = {start: 0.0}
        closed = set()
        came_from = {}
        open_heap = [(math.hypot(goal[0] - start[0], goal[1] - start[1]),
                      0.0, start[0], start[1])]

        while open_heap:
            _, current_g, x, y = heapq.heappop(open_heap)
            if (x, y) in closed:
                continue
            closed.add((x, y))
            if (x, y) == goal:
                return self.reconstruct_path(came_from, start, goal)

            for dx, dy, distance in directions:
                nx, ny = x + dx, y + dy
                if not (xmin <= nx <= xmax and ymin <= ny <= ymax) or (nx, ny) in closed:
                    continue
                if not traversable(nx, ny):
                    continue
                # A diagonal step passes between its two adjacent cardinal
                # cells. Reject it if either cell is 30 degrees or steeper so
                # the path cannot cut through the corner of blocked terrain.
                if (dx != 0 and dy != 0 and
                        (not traversable(x + dx, y) or
                         not traversable(x, y + dy))):
                    continue
                cell_cost = self.terrain_cost[ny, nx]
                obstacle_cost = (0.0 if getattr(self, 'obstacle_cost', None) is None
                                 else float(self.obstacle_cost[ny, nx]))
                step_cost = distance * (
                    1.0 + TERRAIN_COST_WEIGHT * float(cell_cost)
                    + OBSTACLE_COST_WEIGHT * obstacle_cost)
                new_g = current_g + step_cost
                if new_g >= g_score.get((nx, ny), math.inf):
                    continue
                came_from[(nx, ny)] = (x, y)
                g_score[(nx, ny)] = new_g
                heuristic = math.hypot(goal[0] - nx, goal[1] - ny)
                heapq.heappush(
                    open_heap, (new_g + heuristic, new_g, nx, ny))
        return None

    @staticmethod
    def reconstruct_path(came_from, start, goal):
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
        self.active_path = list(grid_path or [])
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        if grid_path:
            for grid_x, grid_y in grid_path:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x, pose.pose.position.y = self.grid_to_world(
                    grid_x, grid_y)
                pose.pose.position.z = float(self.elevation[grid_y, grid_x]) + 0.2
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
        self.path_publisher.publish(message)

    def skip_current_goal(self, reason):
        """Report an unreachable goal, stop its path, and advance once."""
        goal_number = self.current_waypoint_index + 1
        goal = self.current_goal()
        if goal is not None:
            self.get_logger().warning(
                f'Skipping waypoint {goal_number} at '
                f'({goal.position.x:.2f}, {goal.position.y:.2f}): {reason}')
        self.publish_path([])
        self.current_waypoint_index += 1
        self.last_plan_position = None
        self.last_planned_goal_index = None
        if self.current_goal() is None:
            self.get_logger().info('No more reachable waypoint goals')

    def planning_callback(self):
        if self.terrain_cost is None or not self.have_odom or not self.have_waypoints:
            return
        goal = self.current_goal()
        if goal is None:
            self.publish_path([])
            return
        if self.distance_to_goal(goal) <= GOAL_TOLERANCE:
            reached_index = self.current_waypoint_index
            reached_distance = self.distance_to_goal(goal)
            self.current_waypoint_index += 1
            self.last_planned_goal_index = None
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(
                    f'Reached waypoint {reached_index + 1} '
                    f'({reached_distance:.3f} m away); advancing to '
                    f'waypoint {self.current_waypoint_index + 1}')
            else:
                self.get_logger().info(
                    f'Reached final waypoint {reached_index + 1} '
                    f'({reached_distance:.3f} m away)')
            goal = self.current_goal()
            if goal is None:
                self.get_logger().info('All waypoints reached!')
                self.publish_path([])
                return
        if not self.in_world_bounds(self.robot_x, self.robot_y):
            self.publish_path([])
            return
        if not self.in_world_bounds(goal.position.x, goal.position.y):
            self.skip_current_goal('goal is outside the terrain map')
            return

        new_goal = self.last_planned_goal_index != self.current_waypoint_index
        start = self.world_to_grid(self.robot_x, self.robot_y)
        target = self.world_to_grid(goal.position.x, goal.position.y)
        grid_path = None
        if self.active_path and not new_goal and self.last_plan_position is not None:
            nearest = self.remaining_path_index(start)
            blocked = not self.path_section_clear(self.active_path[nearest:])
            if not blocked and not self.replan_requested:
                return
            # Clear the controller's path while the synchronous search runs.
            saved_path = self.active_path
            self.publish_path([])
            self.active_path = saved_path
            partial_start = time.perf_counter()
            grid_path = self.repair_path(start, nearest)
            partial_seconds = time.perf_counter() - partial_start
            if grid_path:
                self.get_logger().info(
                    f'Partial A* replanning time: {partial_seconds:.6f} s '
                    f'(success, {len(grid_path)} combined poses)')
            else:
                self.get_logger().info(
                    f'Partial A* replanning time: {partial_seconds:.6f} s '
                    '(failed; trying full A* replan)')

        self.publish_current_goal(goal)
        if grid_path is None:
            full_plan_start = time.perf_counter()
            grid_path = self.astar(start, target)
            full_plan_seconds = time.perf_counter() - full_plan_start
            plan_kind = ('Initial A* planning time' if new_goal
                         else 'Full A* replanning time')
            result = (f'success, {len(grid_path)} poses' if grid_path
                      else 'failed')
            self.get_logger().info(
                f'{plan_kind}: {full_plan_seconds:.6f} s ({result})')
        self.last_plan_position = (self.robot_x, self.robot_y)
        self.last_planned_goal_index = self.current_waypoint_index
        self.replan_requested = False
        if grid_path is None:
            self.skip_current_goal('no traversable terrain path exists')
            return
        self.get_logger().info(f'Published global path with {len(grid_path)} poses')
        self.publish_path(grid_path)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
