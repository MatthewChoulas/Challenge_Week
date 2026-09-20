#!/usr/bin/env python3
"""Download the requested USGS 1 m DEM crop and build Gazebo terrain assets.

Dependencies: numpy, rasterio, pyproj, pillow, requests.
Run from any directory; output defaults to this package's worlds directory.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from pyproj import Transformer
import rasterio
from rasterio.transform import from_origin
from rasterio.warp import reproject, Resampling
from rasterio.windows import from_bounds
import requests

LAT, LON = 38.42287240335025, -110.78495572815902
PROJECT = 'UT_StatewideSouth_2020_A20'
TILE = f'USGS_1M_12_x51y426_{PROJECT}'
BASE = f'https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/1m/Projects/{PROJECT}'
URL = f'{BASE}/TIFF/{TILE}.tif'
OLDER_BASE = 'https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/1m/Projects/UT_Southern_QL1_2018'
OLDER_TILE = 'USGS_one_meter_x51y426_UT_Southern_QL1_2018'
URLS = [URL, f'{OLDER_BASE}/TIFF/{OLDER_TILE}.tif']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'worlds')
    args = parser.parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    x, y = Transformer.from_crs(4326, 26912, always_xy=True).transform(LON, LAT)
    bounds = (x - 1000, y - 1000, x + 1000, y + 1000)
    # Fetch a small padded source window with HTTP range requests, not the full tile.
    sources = []
    with rasterio.Env(GDAL_DISABLE_READDIR_ON_OPEN='EMPTY_DIR'):
        for url in URLS:
            with rasterio.open(url) as src:
                assert src.crs.to_epsg() == 26912 and src.res == (1.0, 1.0)
                window = from_bounds(bounds[0]-3, bounds[1]-3,
                                     bounds[2]+3, bounds[3]+3, src.transform)
                window = window.round_offsets().round_lengths()
                source = src.read(1, window=window, masked=True).filled(np.nan)
                sources.append((source, src.window_transform(window)))

    def sample(width, transform):
        data = np.full((width, width), np.nan, dtype=np.float32)
        # Prefer 2020 wherever available, use the overlapping 2018 survey for gaps.
        for source, src_transform in sources:
            part = np.full_like(data, np.nan)
            reproject(source, part, src_transform=src_transform, src_crs='EPSG:26912',
                      dst_transform=transform, dst_crs='EPSG:26912',
                      src_nodata=np.nan, dst_nodata=np.nan, resampling=Resampling.bilinear)
            missing = ~np.isfinite(data)
            data[missing] = part[missing]
        if not np.isfinite(data).all():
            raise ValueError('Incomplete coverage of requested square')
        return data

    # Exactly 2000 x 2000 cells, 1 m pixels, centered on the supplied coordinate.
    transform = from_origin(bounds[0], bounds[3], 1, 1)
    dem = sample(2000, transform)
    dem_path = out / 'usgs_utah_dem_1m.tif'
    with rasterio.open(dem_path, 'w', driver='GTiff', width=2000, height=2000,
                       count=1, dtype='float32', crs='EPSG:26912', transform=transform,
                       compress='deflate', predictor=3, tiled=True) as dst:
        dst.write(dem, 1)
        dst.update_tags(source=json.dumps(URLS), vertical_units='meters',
                        center_latitude=LAT, center_longitude=LON)

    # Gazebo uses 2^n+1 vertices; sample inclusive square boundaries (not pixel edges).
    spacing = 2000 / 2048
    vertex_transform = from_origin(bounds[0]-spacing/2, bounds[3]+spacing/2,
                                   spacing, spacing)
    terrain = sample(2049, vertex_transform)
    minimum, maximum = float(terrain.min()), float(terrain.max())
    height_range = maximum - minimum
    center_height = float(terrain[1024, 1024])
    encoded = np.rint((terrain.astype(np.float64)-minimum) / height_range * 65535).astype(np.uint16)
    heightmap = Image.fromarray(encoded)
    heightmap.save(out / 'usgs_utah_heightmap.png')
    # Retain a lightweight preview asset. Gazebo uses the full map with
    # terrain paging enabled so visual and collision elevations agree.
    heightmap.resize((513, 513), Image.Resampling.BILINEAR).save(
        out / 'usgs_utah_visual_heightmap.png')
    Image.new('RGB', (16, 16), (163, 126, 87)).save(out / 'usgs_utah_diffuse.png')
    Image.new('RGB', (16, 16), (128, 128, 255)).save(out / 'usgs_utah_normal.png')
    # A shaded overview for inspection, not an aerial photograph.
    dy, dx = np.gradient(dem, 1.0)
    shade = np.clip((0.5*dx + 0.5*dy + 0.7071) / np.sqrt(dx*dx+dy*dy+1), 0, 1)
    rgb = np.clip(np.array([190, 150, 103]) * (0.3 + 0.7*shade[..., None]), 0, 255).astype(np.uint8)
    Image.fromarray(rgb).resize((1000, 1000)).save(out / 'usgs_utah_preview.png')

    metadata_url = f'{BASE}/metadata/{TILE}.xml'
    response = requests.get(metadata_url, timeout=60)
    response.raise_for_status()
    (out / 'usgs_utah_source_metadata.xml').write_bytes(response.content)
    older_metadata_url = f'{OLDER_BASE}/metadata/{OLDER_TILE}_meta.xml'
    response = requests.get(older_metadata_url, timeout=60)
    response.raise_for_status()
    (out / 'usgs_utah_source_2018_metadata.xml').write_bytes(response.content)
    info = dict(center_wgs84={'latitude': LAT, 'longitude': LON},
                crs='EPSG:26912', vertical_datum='NAVD88', center_utm_m=[x, y], bounds_utm_m=list(bounds),
                extent_m=[2000, 2000], dem_shape=[2000, 2000], dem_pixel_size_m=1,
                dem_elevation_min_m=float(dem.min()), dem_elevation_max_m=float(dem.max()),
                gazebo_vertex_shape=[2049, 2049], gazebo_vertex_spacing_m=spacing,
                gazebo_visual_vertex_shape=[2049, 2049],
                gazebo_visual_vertex_spacing_m=2000/2048,
                gazebo_min_elevation_m=minimum, gazebo_max_elevation_m=maximum,
                center_elevation_m=center_height,
                gazebo_height_range_m=height_range, gazebo_z_offset_m=minimum-center_height,
                png_quantization_step_m=height_range/65535,
                gazebo_axes='x = UTM grid east, y = UTM grid north, z = up; center ground at z=0',
                resampling='bilinear; 1 m source; exact centered UTM square',
                source_urls=URLS, source_metadata_urls=[metadata_url, older_metadata_url],
                source_priority='2020 first, 2018 where 2020 has NoData; no synthetic gap filling',
                generated_utc=datetime.now(timezone.utc).isoformat(),
                dem_sha256=hashlib.sha256(dem_path.read_bytes()).hexdigest())
    (out / 'usgs_utah_metadata.json').write_text(json.dumps(info, indent=2)+'\n')
    # Ogre terrain ignores model poses; DART ignores heightmap pos.
    # Keep parents at the origin and place visual and collision separately.
    geometry = f'''<geometry><heightmap>
            <uri>{{uri}}</uri>
            <size>2000 2000 {height_range:.9f}</size>
            <pos>0 0 {{offset}}</pos>
            <sampling>1</sampling>
            {{extra}}
          </heightmap></geometry>'''
    texture = '''<texture><size>100</size>
              <diffuse>usgs_utah_diffuse.png</diffuse>
              <normal>usgs_utah_normal.png</normal>
            </texture><use_terrain_paging>true</use_terrain_paging>'''
    world = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="usgs_utah">
    <physics name="physics" type="ignored">
      <max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
      <dart><collision_detector>bullet</collision_detector></dart>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.65 0.8 0.95 1</background><shadows>false</shadows></scene>
    <gui fullscreen="false">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui><title>3D View</title><property type="string" key="state">docked</property></gz-gui>
        <engine>ogre</engine><scene>scene</scene>
        <camera_pose>0 -1600 1400 0 0.65 1.57079632679</camera_pose>
        <camera_clip><near>0.1</near><far>10000</far></camera_clip>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager"/>
      <plugin filename="InteractiveViewControl" name="Interactive view control"/>
      <plugin filename="EntityTree" name="Entity tree"/>
      <plugin filename="WorldControl" name="World control"/>
    </gui>
    <light name="sun" type="directional">
      <pose>0 0 1500 0 0 0</pose><cast_shadows>true</cast_shadows>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.5 -0.7</direction>
    </light>
    <model name="usgs_utah_terrain"><static>true</static>
      <pose>0 0 0 0 0 0</pose><link name="terrain">
      <collision name="terrain_collision">
        <pose>0 0 {minimum-center_height:.9f} 0 0 0</pose>
        {geometry.format(uri='usgs_utah_heightmap.png', extra='', offset=0)}</collision>
      <visual name="terrain_visual">{geometry.format(uri='usgs_utah_heightmap.png', extra=texture, offset=f'{minimum-center_height:.9f}')}</visual>
    </link></model>
  </world>
</sdf>
'''
    (out / 'usgs_utah.sdf').write_text(world)
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
