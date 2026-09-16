import math
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from scipy.ndimage import distance_transform_edt

class LidarMapper(Node):

    def __init__(self):
        super().__init__('lidar_mapper')

        self.cell_len = 0.1  # meters
        self.map_width = 300 # cells
        self.map_height = 300 # cells

        self.origin_x = -15.0
        self.origin_y = -15.0

        self.free_space_update = -0.25
        self.occupied_space_update = 1.2

        self.min_odds = -5.0
        self.max_odds = 5.0

        self.free_probability = 0.35
        self.occupied_probability = 0.65
        self.occupied_clear_probability = 0.45

        self.log_odds = np.zeros((self.map_height, self.map_width), dtype=np.float32)

        self.occupied_state = np.zeros((self.map_height, self.map_width), dtype=bool)

        self.observed = np.zeros((self.map_height, self.map_width,), dtype=bool)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.robot_radius = 0.18
        self.safety_margin = 0.05
        self.obstacle_influence_radius = 4.0
        
        self.unknown_cell_cost = 3


        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/model/robot/odometry',
            self.odom_callback,
            10
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        self.map_publisher_timer = self.create_timer(
            0.1,
            self.publish_map
        )

        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            '/costmap',
            map_qos
        )

        self.get_logger().info('LiDAR mapper started')

    def odom_callback(self, msg):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.robot_x = position.x
        self.robot_y = position.y

        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)

        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def world_to_grid(self, x, y):
        grid_x = int(math.floor((x - self.origin_x) / self.cell_len))
        grid_y = int(math.floor((y - self.origin_y) / self.cell_len))

        return grid_x, grid_y

    def in_grid_bounds(self, x, y):
        if x < self.map_width and x >= 0 and y < self.map_height and y >= 0:
            return True

        return False

    def ray_tracing(self, x0, y0, x1, y1):
        cells = []

        dx = x1 - x0
        dy = y1 - y0

        sx = 1 if dx > 0 else (-1 if dx < 0 else 0)
        sy = 1 if dy > 0 else (-1 if dy < 0 else 0)

        nx = abs(dx)
        ny = abs(dy)

        x = x0
        y = y0

        ix = 0
        iy = 0

        cells.append((x, y))

        while ix < nx or iy < ny:

            px = (1 + 2 * ix) * ny
            py = (1 + 2 * iy) * nx

            if px == py:

                if sx != 0:
                    cell = (x + sx, y)
                    cells.append(cell)

                if sy != 0:
                    cell = (x, y + sy)
                    cells.append(cell)

                x += sx
                y += sy

                ix += 1
                iy += 1

                cell = (x, y)
                cells.append(cell)

            elif px < py:
                x += sx
                ix += 1

                cell = (x, y)
                cells.append(cell)

            else:
                y += sy
                iy += 1

                cell = (x, y)
                cells.append(cell)

        return cells


    def scan_callback(self, scan):

        robot_grid_x, robot_grid_y = self.world_to_grid(self.robot_x, self.robot_y)

        if not self.in_grid_bounds(robot_grid_x, robot_grid_y):
            return

        angle = scan.angle_min

        for r in scan.ranges:
            if math.isnan(r):
                angle += scan.angle_increment
                continue

            hit_obstacle = (math.isfinite(r) and r >= scan.range_min and r < scan.range_max)

            if hit_obstacle:
                ray_range = r
            else:
                ray_range = scan.range_max

            global_angle = self.robot_yaw + angle

            end_x = self.robot_x + ray_range * math.cos(global_angle)
            end_y = self.robot_y + ray_range * math.sin(global_angle)

            end_grid_x, end_grid_y = self.world_to_grid(end_x, end_y)

            end_grid_x = max(0, min(self.map_width - 1, end_grid_x))
            end_grid_y = max(0, min(self.map_height - 1, end_grid_y))

            cells = self.ray_tracing(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y)

            if len(cells) == 0:
                angle += scan.angle_increment
                continue

            free_cells = cells[:-1] if hit_obstacle else cells

            for grid_x, grid_y in free_cells:
                if not self.in_grid_bounds(grid_x, grid_y):
                    continue


                if self.occupied_state[grid_y, grid_x]:
                    # Takes many scans to clear a known obstacle
                    update = -0.01
                else:
                    update = self.free_space_update

                self.log_odds[grid_y, grid_x] += update
                self.log_odds[grid_y, grid_x] = np.clip(self.log_odds[grid_y, grid_x], self.min_odds, self.max_odds)

                self.observed[grid_y, grid_x] = True
            
            if hit_obstacle:
                grid_x, grid_y = cells[-1]

                if self.in_grid_bounds(grid_x, grid_y):
                    self.log_odds[grid_y, grid_x] += self.occupied_space_update
                    self.log_odds[grid_y, grid_x] = np.clip(self.log_odds[grid_y, grid_x], self.min_odds, self.max_odds)
                    self.observed[grid_y, grid_x] = True

            angle += scan.angle_increment

        self.get_logger().info(
            f"Scan: {len(scan.ranges)} rays, "
            f"robot=({self.robot_x:.2f}, {self.robot_y:.2f}), "
            f"yaw={self.robot_yaw:.2f}"
        )

    def publish_map(self):

        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.cell_len
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        probabilities = 1.0 / (1.0 + np.exp(-self.log_odds))

        grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        self.occupied_state[probabilities >= self.occupied_probability] = True

        self.occupied_state[probabilities <= self.occupied_clear_probability] = False

        grid[self.observed & (probabilities < self.free_probability)] = 0

        grid[self.occupied_state] = 100
        
        free_mask = (self.observed & ~self.occupied_state & (probabilities <= self.free_probability))

        grid[free_mask] = 0

        msg.data = grid.flatten().tolist()

        known = np.count_nonzero(grid != -1)
        free = np.count_nonzero(grid == 0)
        occupied = np.count_nonzero(grid == 100)

        self.get_logger().info(f"known={known}, free={free}, occupied={occupied}")

        self.map_publisher.publish(msg)


        costmap = self.generate_costmap(grid)
        self.publish_costmap(costmap, msg.header.stamp)

    def generate_costmap(self, occupancy_grid):
        occupied = occupancy_grid == 100
        unknown = occupancy_grid == -1

        # distance transform edt calculates distance from any nonzero cell to the closests zero cell
        # ~occupied makes any obstacle cell 0 and all non obstacle cells 1 
        if np.any(occupied):
            distance = distance_transform_edt(~occupied)
            distance *= self.cell_len
        else:
            distance = np.full(occupancy_grid.shape, np.inf, dtype=np.float32)

        # Initially everything has zero cost
        costmap = np.zeros(occupancy_grid.shape, dtype=np.int8)

        # determine minimum and maximum distances to setup the cost scale
        collision_radius = self.robot_radius + self.safety_margin
        influence_mask = (distance > collision_radius) & (distance < self.obstacle_influence_radius)

        # Normalize and scale to range from 1-99
        normalized_cost = (self.obstacle_influence_radius - distance[influence_mask]) / (self.obstacle_influence_radius - collision_radius)
        costmap[influence_mask] = (1 + 98 * normalized_cost).astype(np.int8)

        # set all collision cells to 100
        collision_mask = distance <= collision_radius
        costmap[collision_mask] = 100

        # set cost for unknown cells
        unknown_safe_cells  = unknown & (costmap < 100)
        costmap[unknown_safe_cells] = np.maximum(costmap[unknown_safe_cells], self.unknown_cell_cost)

        return costmap

    def publish_costmap(self, costmap, stamp):

        msg = OccupancyGrid()

        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.cell_len
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = costmap.flatten().tolist()

        self.costmap_publisher.publish(msg)



def main():
    rclpy.init()

    node = LidarMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

        

        