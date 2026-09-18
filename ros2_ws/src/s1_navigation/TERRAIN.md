# USGS Utah terrain

A 2,000 × 2,000 m square centered at WGS84 latitude **38.42287240335025**,
longitude **-110.78495572815902**. The square follows the NAD83 / UTM zone 12N
(EPSG:26912) grid; its dimensions are projected meters.

## Open in Gazebo

From the workspace root (the package has already been built):

```bash
source install/setup.bash
ros2 launch s1_navigation terrain.launch.py
```

For server only, append `gui:=false`. Close the terrain window or press Ctrl+C to
stop. This launches the terrain and a terrain-constrained robot at the requested geographic center.
The camera starts near the robot. The ROS bridge provides velocity commands, full
3D odometry, lidar scans and TF.

After copying to another workspace, first run:

```bash
colcon build --packages-select s1_planar_drive s1_navigation --symlink-install
```

## Files in `worlds/`

- `usgs_utah_dem_1m.tif`: 2,000 × 2,000 float32 GeoTIFF cells at 1 m spacing;
  absolute elevations in meters NAVD88. All cells contain valid elevations.
- `usgs_utah.sdf`: Gazebo Harmonic terrain world.
- `usgs_utah_heightmap.png`: 16-bit grayscale, 2,049 × 2,049 vertices, resampled
  from the 1 m source with bilinear interpolation for Gazebo's `2^n + 1` requirement.
  Inclusive vertex spacing is 2000/2048 = 0.9765625 m; this does not add source detail.
- `usgs_utah_visual_heightmap.png`: 513 × 513 display mesh. Physics retains the
  full-resolution heightmap; the lighter visual mesh keeps the Gazebo window responsive.
- `usgs_utah_diffuse.png`, `usgs_utah_normal.png`: simple terrain material.
- `usgs_utah_preview.png`: shaded elevation overview, north up; not aerial imagery.
- `usgs_utah_metadata.json`: bounds, scale, datum, source URLs and DEM checksum.
- `usgs_utah_source_metadata.xml`, `usgs_utah_source_2018_metadata.xml`: USGS metadata.

USGS source tiles:

- [Utah Statewide South 2020](https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/1m/Projects/UT_StatewideSouth_2020_A20/TIFF/USGS_1M_12_x51y426_UT_StatewideSouth_2020_A20.tif)
- [Utah Southern QL1 2018](https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/1m/Projects/UT_Southern_QL1_2018/TIFF/USGS_one_meter_x51y426_UT_Southern_QL1_2018.tif)

The 2020 survey takes priority. Its missing areas are supplied by the 2018 survey;
together they cover the complete square. No missing area was filled with invented
terrain. USGS cautions that different survey projects are not necessarily seamless,
so a survey boundary can remain in the combined DEM. Both use 1 m source pixels.
These are bare-earth elevations, not buildings or vegetation; 1 m pixel spacing is
not a claim of 1 m vertical accuracy.

Elevation spans approximately **1357.42–1430.59 m NAVD88**, about **73.17 m** of
relief. The supplied center is approximately **1375.53 m NAVD88**. In Gazebo,
`x=0, y=0` is that center, positive X is UTM grid east, positive Y is grid north,
and the center's ground elevation is shifted to approximately Z=0 (PNG rounding
is below 0.6 mm). Vertical exaggeration is 1×. Retain the JSON scale and offset
when reusing the PNG: its grayscale values are normalized, not absolute meters.

## Robot driving

The terrain robot has a 1.0 × 1.0 × 0.25 m, 5 kg body and four 0.15 m spherical
supports. Its terrain-aware holonomic drive samples the full-resolution DEM and
keeps every support above the surface, preventing launches and terrain tunneling.
It can drive forward, sideways, and turn through `/model/robot/cmd_vel`:

```bash
# In a second terminal, after sourcing install/setup.bash:
ros2 topic pub -r 20 /model/robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

Press Ctrl+C to stop publishing; the drive requests zero speed after 0.5 seconds
without a command. Use `linear.y` to strafe and `angular.z` to turn. Total speed
is capped at 100 m/s and turn rate at 0.6 rad/s. The drive checks intermediate
positions and all four supports; motion stops when any support reaches terrain
steeper than 30 degrees. This is an idealized omnidirectional rover, not a tire,
suspension, soil-traction, or rollover simulation. The 0.5 second command
watchdog stops motion immediately when commands cease.

`ros2 launch s1_navigation terrain.launch.py follow_path:=true` enables the existing
path controller for paths published to `/planned_path`. Do not also publish manual
velocity commands in that mode. The old 30 m flat-ground mapper, random waypoint
publisher, and planner are not launched: terrain-aware autonomous route planning
is a separate feature. `simulation.launch.py` still runs the original flat course.

Use `gui:=false sensors:=false` for a physics-only run without graphics. With sensors
enabled, `/scan` contains the existing 720-ray lidar and `/model/robot/odometry`
contains Z, roll, pitch and yaw as well as X/Y. The lidar tilts with the robot.

The world is composed at launch from `usgs_utah.sdf` and `terrain_robot.sdf`.
The terrain model stays at the origin. Its vertical offset is applied to the
collision's `<pose>` for DART and independently to the visual heightmap's `<pos>`
for Ogre. Ogre terrain ignores ancestor model poses, while DART ignores the
heightmap-specific position. Both surfaces now use the same elevation offset.
The world grid is at Z=0, matching ground level at the requested center; a flat
grid naturally intersects uneven terrain elsewhere.
Regenerating the DEM preserves this correction and does not remove the robot.

## Regenerate

`tools/prepare_usgs_terrain.py` downloads source windows with HTTP range requests,
checks full coverage, crops the exact centered square, and recreates the assets.
It requires Python with `numpy rasterio pyproj pillow requests` and network access.

```bash
python3 src/s1_navigation/tools/prepare_usgs_terrain.py
```

Dependencies used for initial generation were installed only in the temporary
`/tmp/usgs-dem-venv` environment; running Gazebo does not require them.

## Validation on this machine

Verified raster bounds, dimensions, valid data coverage and PNG elevation scale;
`gz sdf -k` passed; the package built successfully; Gazebo ran the terrain physics
and loaded its visual textures. The viewer uses the `ogre` renderer because the
local `ogre2` configuration reported graphics errors during validation.

Robot integration verification (requires NumPy and Pillow in the ROS Python environment):

```bash
python3 src/s1_navigation/tools/check_terrain_robot.py --gui
```

This starts an isolated test simulation, verifies ground clearance, elevation
changes, forward/sideways motion, turning, command timeout and lidar, then closes it.
Omit `--gui` for the physics and motion checks alone.

The robot check passed with forward and sideways travel, turning, command timeout,
and approximately 0.32–0.34 m body-center clearance above the measured surface.
A separate 100 m/s test stopped at an over-30-degree terrain face and remained at
the same coordinates under repeated commands. The focused world, path-tracking,
and launch regression suite also passes.
