# Terrain GPS waypoints

The terrain launch starts two simple nodes. The original flat-world
`waypoint_publisher.py` remains in use by `simulation.launch.py`.

- `gps_waypoint_publisher` samples three points in the local square
  `[-1000, 1000] × [-1000, 1000]` metres. It rejects points too close to the
  fixed GPS origin or another goal. Defaults are 200 metres for both distances.
  It uses WGS84 `Geod.fwd(origin_lon, origin_lat, bearing, distance)` to turn
  the points into GPS coordinates. Bearing is `atan2(east, north)` in degrees.
- `/gps_waypoints` is `std_msgs/Float64MultiArray`: exactly six values
  `[lat1, lon1, lat2, lon2, lat3, lon3]`, in degrees. Its layout labels identify
  the three rows and latitude/longitude columns. The same list is repeated;
  waypoints are generated only once per launch.
- `gps_waypoint_converter` subscribes to that list. `Geod.inv` gives bearing
  and distance from the fixed GPS origin. Local `x = distance*sin(bearing)`
  and `y = distance*cos(bearing)`, matching the GPS robot reset. It samples
  the visual terrain heightmap for `z` and publishes `/waypoints` (`geometry_msgs/PoseArray`,
  frame `odom`) and `/waypoint_markers` (`visualization_msgs/MarkerArray`).
- The converter calls Gazebo's bridged `/world/usgs_utah/create` service to
  create three static green posts named `gps_waypoint_1` through `3`. Each
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
The local square uses the same geodesic coordinate convention as the rover;
its east/north axes differ slightly from the source DEM's UTM grid axes.
Changing the GPS topic after the first list is accepted does not replace goals;
restart the terrain launch to generate and display a new set.

```bash
ros2 topic echo /gps_waypoints --once
ros2 topic echo /waypoints --once --qos-durability transient_local
```
