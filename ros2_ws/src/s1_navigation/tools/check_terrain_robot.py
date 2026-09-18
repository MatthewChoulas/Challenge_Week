#!/usr/bin/env python3
"""Integration check: real Gazebo contacts, motion, odometry and command timeout.

Run after sourcing install/setup.bash. Add --gui to check the viewer and lidar.
Uses an isolated ROS domain and always shuts down its own simulation.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
from PIL import Image
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    worlds = Path(__file__).resolve().parents[1] / 'worlds'
    metadata = json.loads((worlds / 'usgs_utah_metadata.json').read_text())
    heights = np.asarray(Image.open(worlds / 'usgs_utah_heightmap.png'), dtype=float)
    heights *= metadata['gazebo_height_range_m'] / 65535
    heights += metadata['gazebo_z_offset_m']
    spacing = metadata['gazebo_vertex_spacing_m']
    rclpy.init()
    node = rclpy.create_node('terrain_motion_check')
    odom, scans = [], []
    node.create_subscription(Odometry, '/model/robot/odometry', lambda m: odom.append(m), 100)
    node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)
    pub = node.create_publisher(Twist, '/model/robot/cmd_vel', 10)
    log_path = Path(tempfile.gettempdir()) / f's1_terrain_check_{os.getpid()}.log'
    log = log_path.open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 's1_navigation', 'terrain.launch.py',
         f'gui:={str(args.gui).lower()}', f'sensors:={str(args.gui).lower()}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def sim_time():
        stamp = odom[-1].header.stamp
        return stamp.sec + stamp.nanosec / 1e9

    def step(seconds, x=0.0, y=0.0, turn=0.0, publish=True):
        start = sim_time() if odom else None
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'Gazebo exited; see {log_path}')
            if publish:
                cmd = Twist()
                cmd.linear.x, cmd.linear.y, cmd.angular.z = x, y, turn
                pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
            if odom:
                if start is None:
                    start = sim_time()
                if sim_time() - start >= seconds:
                    return
        raise TimeoutError(f'No simulation progress; see {log_path}')

    def state(label):
        msg = odom[-1]
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        # Pillow indexes rows from the top; Gazebo's image sampler indexes Y
        # from the bottom, so positive world Y decreases the raw PNG row.
        col, row = 1024 + p.x / spacing, 1024 - p.y / spacing
        i, j = int(row), int(col)
        a, b = row-i, col-j
        ground = ((1-a)*((1-b)*heights[i,j]+b*heights[i,j+1]) +
                  a*((1-b)*heights[i+1,j]+b*heights[i+1,j+1]))
        clearance = p.z - ground
        assert 0.28 < clearance < 0.42, (label, 'Ground contact lost', clearance)
        print(f'{label}: xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), '
              f'yaw={yaw:.3f}, ground clearance={clearance:.3f} m', flush=True)
        return np.array([p.x, p.y, p.z]), yaw

    try:
        step(3.0)
        initial, _ = state('Settled')
        step(8.0, x=0.4)
        forward, _ = state('Forward')
        assert forward[0] - initial[0] > 2.0
        assert abs(forward[2] - initial[2]) > 0.02, 'Height did not follow terrain'
        step(2.0, publish=False)
        stopped, _ = state('Command timeout')
        v = odom[-1].twist.twist.linear
        assert math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) < 0.03, 'Watchdog did not stop'
        step(4.0, y=0.3)
        sideways, yaw_before = state('Sideways')
        assert np.linalg.norm(sideways[:2] - stopped[:2]) > 0.7
        step(3.0, turn=0.3)
        _, yaw_after = state('Turn')
        turn_angle = math.atan2(math.sin(yaw_after-yaw_before), math.cos(yaw_after-yaw_before))
        assert turn_angle > 0.45, ('Insufficient turn', turn_angle)
        step(2.0)
        state('Stopped')
        if args.gui:
            assert scans and len(scans[-1].ranges) == 720, 'Missing lidar scan'
            print(f'Lidar: {len(scans[-1].ranges)} rays', flush=True)
        print(f'PASS. Gazebo log: {log_path}', flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
