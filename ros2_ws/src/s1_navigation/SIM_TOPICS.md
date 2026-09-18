# Original simulation topics

This is the ROS topic interface for the flat obstacle-course simulation started with:

```bash
source install/setup.bash
ros2 launch s1_navigation simulation.launch.py
```

The list below describes the ROS side of the Gazebo bridge. Topic names are absolute.

## Published

| Topic | Type | Publisher | Purpose |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo bridge | Simulation time |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo bridge | 720-ray lidar scan |
| `/model/robot/odometry` | `nav_msgs/msg/Odometry` | Gazebo bridge | Robot pose and velocity |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `lidar_mapper` | Lidar-built occupancy map |
| `/costmap` | `nav_msgs/msg/OccupancyGrid` | `lidar_mapper` | Planner costmap |
| `/waypoints` | `geometry_msgs/msg/PoseArray` | `waypoint_publisher` | Random navigation goals |
| `/waypoint_markers` | `visualization_msgs/msg/MarkerArray` | `waypoint_publisher` | RViz/Gazebo waypoint markers |
| `/planned_path` | `nav_msgs/msg/Path` | `astar_planner` | A* path through the costmap |
| `/current_goal` | `geometry_msgs/msg/PoseStamped` | `astar_planner` | Goal currently being planned toward |
| `/model/robot/cmd_vel` | `geometry_msgs/msg/Twist` | `path_controller` | Robot velocity command |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom_tf_broadcaster` | `odom` → `base_link` transform |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `static_transform_publisher` | `base_link` → lidar transform |

## Subscribed

| Topic | Type | Subscribers | Purpose |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | `lidar_mapper` | Adds lidar rays to the map |
| `/model/robot/odometry` | `nav_msgs/msg/Odometry` | `lidar_mapper`, `astar_planner`, `path_controller`, `odom_tf_broadcaster` | Updates pose, planning, control, and TF |
| `/costmap` | `nav_msgs/msg/OccupancyGrid` | `astar_planner` | Supplies obstacle costs for A* |
| `/waypoints` | `geometry_msgs/msg/PoseArray` | `astar_planner` | Supplies navigation goals |
| `/planned_path` | `nav_msgs/msg/Path` | `path_controller` | Supplies the path to follow |
| `/model/robot/cmd_vel` | `geometry_msgs/msg/Twist` | Gazebo bridge / `PlanarDrive` | Drives the robot in Gazebo |

`/model/robot/cmd_vel` can also be published manually. If `path_controller` is
running, it may publish its own commands on the same topic.

The original simulation uses `use_sim_time`, so nodes should use `/clock` rather
than wall time when replaying or synchronizing simulation data.
