"""Drive the free-body, holonomic S1 robot on the USGS terrain."""
import os
import shutil
import sys
import tempfile
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from s1_navigation.terrain_world import create_robot_world


def launch_terrain(context):
    worlds = os.path.join(get_package_share_directory('s1_navigation'), 'worlds')
    sensors = LaunchConfiguration('sensors').perform(context).lower()
    if sensors not in ('true', 'false'):
        raise ValueError('sensors must be true or false')
    run_dir = tempfile.mkdtemp(prefix='s1_terrain_')
    world = create_robot_world(worlds, os.path.join(run_dir, 'terrain_robot_world.sdf'),
                               sensors=sensors == 'true')
    supervisor = [sys.executable, '-m', 's1_navigation.managed_process']
    prefix = f'{sys.executable} -m s1_navigation.managed_process'
    gazebo_gui = os.path.join(
        get_package_prefix('s1_planar_drive'), 'bin', 's1_gazebo_gui')

    def cleanup(_context):
        shutil.rmtree(run_dir, ignore_errors=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        LogInfo(msg='Terrain robot: /model/robot/cmd_vel; forward X, sideways Y, turn Z.'),
        SetEnvironmentVariable('GZ_PARTITION', f'usgs_terrain_{uuid.uuid4().hex}'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', worlds + os.pathsep +
                               os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        ExecuteProcess(
            cmd=supervisor + ['gz', 'sim', '-s', '-r', world, '--force-version', '8'],
            name='terrain_server', output='screen',
            on_exit=Shutdown(reason='Terrain server exited'),
            sigterm_timeout='5', sigkill_timeout='5'),
        ExecuteProcess(
            cmd=supervisor + [gazebo_gui],
            name='terrain_gui', output='screen', condition=IfCondition(LaunchConfiguration('gui')),
            on_exit=Shutdown(reason='Terrain window closed'),
            sigterm_timeout='5', sigkill_timeout='5'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='odom_tf_broadcaster',
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--z', '0.30', '--frame-id', 'base_link',
                        '--child-frame-id', 'robot/base_link/lidar'],
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='path_controller',
             condition=IfCondition(LaunchConfiguration('follow_path')),
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open Gazebo viewer'),
        DeclareLaunchArgument('sensors', default_value='true', description='Render the lidar'),
        DeclareLaunchArgument('follow_path', default_value='false',
                              description=(
                                  'Follow /planned_path instead of manual '
                                  'velocity commands')),
        OpaqueFunction(function=launch_terrain),
    ])
