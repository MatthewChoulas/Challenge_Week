# GPS start location and local odometry

Build and source the package, then run the two nodes in separate terminals:

```bash
colcon build --packages-select s1_navigation --symlink-install
source install/setup.bash
ros2 run s1_navigation gps_start_publisher
ros2 run s1_navigation gps_odometry
```

The publisher sends a constant `sensor_msgs/msg/NavSatFix` on
`/robot_gps_start_location`. Its default position is the DEM origin:

```text
latitude:  38.42287240335025 degrees
longitude: -110.78495572815902 degrees
altitude:  0 metres (configurable)
```

The publisher is parameterized, so a different test fix can be sent without
editing code:

```bash
ros2 run s1_navigation gps_start_publisher --ros-args \
  -p latitude:=38.423 -p longitude:=-110.784
```

When the node is already running, parameters can be changed live:

```bash
ros2 param set /gps_start_publisher latitude 38.423
ros2 param set /gps_start_publisher longitude -110.784
ros2 param set /gps_start_publisher altitude 12.0
```

The publisher sends the updated fix on its next timer cycle, and
`gps_odometry` publishes the corresponding local position.

`gps_odometry` converts each changed GPS start fix to east/north coordinates
and calls `/world/usgs_utah/set_pose` to move the Gazebo model. It queues fixes
until raw odometry and the service are available and retries rejected requests.
Repeated identical fixes do not reset a moving rover.

Gazebo odometry is bridged to `/model/robot/odometry_raw` and relayed unchanged
on `/model/robot/odometry` for the TF broadcaster and RViz. There is no extra
position offset: both displays use the actual simulator position. TerrainDrive
places the rover on the DEM at the requested XY and determines its height and
tilt. GPS altitude does not override terrain height in this mode.

## Coordinate transformation

The subscriber uses `pyproj.Geod(ellps="WGS84")` to calculate the shortest
geodesic from the origin to each fix on the WGS 84 reference ellipsoid. The
geodesic inverse calculation returns an initial azimuth and distance in metres:

```text
(azimuth, distance) = WGS84_GEODESIC(origin, fix)
east  = distance * sin(azimuth)
north = distance * cos(azimuth)
up    = fix_altitude - origin_altitude
```

The azimuth is measured clockwise from north, so sine produces the east
component and cosine produces the north component. Therefore, local `x` points
east, local `y` points north, and local `z` is the ellipsoidal height difference
from the origin. If the incoming fix equals the origin, the published local
position is `(0, 0, 0)`.

The conversion is a position reference, not a complete navigation filter. GPS
noise, antenna offsets, heading, velocity estimation, and sensor fusion would
need to be added for production localization. The node preserves Gazebo orientation, velocity, timestamps, and covariance.

The origin can be changed at runtime without editing the node:

```bash
ros2 run s1_navigation gps_odometry --ros-args \
  -p origin_latitude:=38.42287240335025 \
  -p origin_longitude:=-110.78495572815902 \
  -p origin_altitude:=0.0
```
