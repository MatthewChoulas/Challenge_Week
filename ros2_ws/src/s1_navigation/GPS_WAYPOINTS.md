# Terrain GPS waypoints

The terrain launch starts `gps_waypoint_converter`; the GUI mission manager
normally publishes its GPS input. The standalone `gps_waypoint_publisher` can
still generate a three-goal test mission. The original flat-world
`waypoint_publisher.py` remains in use by `simulation.launch.py`.

- `gps_waypoint_publisher` samples three points in the local square
  `[-1000, 1000] × [-1000, 1000]` metres. It rejects points too close to the
  fixed GPS origin or another goal. Defaults are 200 metres for both distances.
  It adds each local point to the origin's NAD83 / UTM Zone 12N
  (`EPSG:26912`) coordinate and transforms the result to WGS 84 latitude and
  longitude.
- `/gps_waypoints` is `std_msgs/Float64MultiArray` containing one or more
  `[latitude, longitude]` pairs in degrees. The standalone publisher emits
  three pairs and repeats the same generated list.
- `gps_waypoint_converter` subscribes to that list. It projects each WGS 84
  coordinate to EPSG:26912, then subtracts the projected origin. The resulting
  local `x` is grid easting and `y` is grid northing, matching the GPS robot
  reset and terrain grid. It samples
  the visual terrain heightmap for `z` and publishes `/waypoints` (`geometry_msgs/PoseArray`,
  frame `odom`) and `/waypoint_markers` (`visualization_msgs/MarkerArray`).
- The converter calls Gazebo's bridged `/world/usgs_utah/create` service to
  create one static green post per goal, named `gps_waypoint_1`, and so on. Each
  post is 2 m wide, with its bottom at world Z=0. The visual terrain heightmap
  is sampled at each waypoint: cylinder length is `terrain_z + 20` and centre
  Z is `(terrain_z + 20) / 2`. Thus each top is 20 m above the sampled terrain.
  Gazebo and RViz use the same calculation. On terrain below Z=0 the base
  floats above the surface, as required by the fixed zero base; its top still
  has 20 m clearance. Clearance is measured at the waypoint centre; sloped
  ground can vary across the cylinder width.
  These are visual objects without collision geometry. Failed creation is retried.

Both ROS outputs retain their latest message for late subscribers. In RViz,
set Fixed Frame to `odom`, add a MarkerArray display on `/waypoint_markers`,
and select Transient Local durability. Gazebo displays the posts automatically.

```bash
source install/setup.bash
ros2 launch s1_navigation terrain.launch.py \
  waypoint_start_distance:=200.0 waypoint_spacing:=300.0
```

Use `waypoint_seed:=42` for repeatable goals; the default `-1` gives a fresh
set each launch. Impossible distance constraints fail after 10,000 attempts
with an explanatory error. There is no obstacle, slope, or reachability filter.

The exclusion centre is the fixed origin
`38.42287240335025, -110.78495572815902`, not a later GPS teleport position.
The local square uses the same UTM coordinate convention as the rover and the
source DEM, so their east/north axes coincide.
Changing the GPS topic after the first list is accepted does not replace goals;
restart the terrain launch to generate and display a new set.

```bash
ros2 topic echo /gps_waypoints --once
ros2 topic echo /waypoints --once --qos-durability transient_local
```
