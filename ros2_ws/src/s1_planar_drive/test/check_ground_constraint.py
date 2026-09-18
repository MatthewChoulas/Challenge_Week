"""Headless collision/rotation smoke test; run after sourcing install/setup.bash."""
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET

os.environ['GZ_PARTITION'] = 's1_ground_test_' + uuid.uuid4().hex
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist

root = Path(__file__).resolve().parents[2]
world = ET.parse(root / 's1_navigation/worlds/s1_world_template.sdf')
w = world.getroot().find('world')
for plugin in list(w.findall('plugin')):
    if plugin.get('name') == 'gz::sim::systems::Sensors':
        w.remove(plugin)
base = w.find("model[@name='robot']/link[@name='base_link']")
base.remove(base.find('sensor'))
ET.SubElement(w, 'model', name='contact_obstacle')
rock = w.find("model[@name='contact_obstacle']")
rock.extend(ET.fromstring('''<elements><static>true</static><pose>1 0 0 0 0 0</pose>
<link name="link"><collision name="collision"><geometry><sphere><radius>0.5</radius>
</sphere></geometry></collision></link></elements>'''))
off_center = '--off-center' in sys.argv
if off_center:
    rock.find('pose').text = '1 0.15 0 0 0 0'
poses = []
lock = threading.Lock()

def on_pose(msg):
    for pose in msg.pose:
        if pose.name == 'robot':
            with lock:
                poses.append((pose.position.x, pose.position.y, pose.position.z,
                              pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w))

node = Node()
node.subscribe(Pose_V, '/world/s1_world/dynamic_pose/info', on_pose)
pub = node.advertise('/model/robot/cmd_vel', Twist)
with tempfile.TemporaryDirectory(prefix='s1_ground_check_') as directory:
    file = Path(directory) / 'world.sdf'
    world.write(file)
    with open(Path(directory) / 'gazebo.log', 'w+') as log:
        process = subprocess.Popen([sys.executable, '-m', 's1_navigation.managed_process',
                                    'gz', 'sim', '-s', '-r', str(file)],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while not poses and time.monotonic() < deadline:
                time.sleep(0.1)
            assert poses, 'No Gazebo pose samples'
            cmd = Twist()
            cmd.linear.x = 0.5
            until = time.monotonic() + 5
            while time.monotonic() < until:
                pub.publish(cmd)
                time.sleep(0.05)
            with lock:
                collision_poses = list(poses)
            assert max(p[0] for p in collision_poses) > 0.25, 'Robot did not move'
            print('Collision end pose:', collision_poses[-1], flush=True)
            if off_center:
                assert max(abs(p[5]) for p in collision_poses) > 1e-4, 'Contact did not allow yaw'
            else:
                assert max(p[0] for p in collision_poses) < 0.8, 'Robot passed through obstacle'
            cmd.linear.x = 0
            cmd.angular.z = 0.5
            until = time.monotonic() + 2
            while time.monotonic() < until:
                pub.publish(cmd)
                time.sleep(0.05)
            with lock:
                samples = list(poses)
            assert len(samples) > 20
            max_height_error = max(abs(p[2] - 0.125) for p in samples)
            max_tilt_quaternion = max(math.hypot(p[3], p[4]) for p in samples)
            assert max_height_error < 1e-4, max_height_error
            assert max_tilt_quaternion < 1e-4, max_tilt_quaternion
            assert abs(samples[-1][5]) > 0.1, 'Yaw motion was locked'
            print(f'PASS: {len(samples)} poses; max height error={max_height_error:.3g} m; '
                  f'max roll/pitch quaternion norm={max_tilt_quaternion:.3g}; '
                  f'ground constraint and yaw motion verified (off_center={off_center})')
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read()[-6000:])
            raise
        finally:
            process.terminate()
            process.wait(timeout=8)
