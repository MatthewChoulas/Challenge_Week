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
    base_seed = int(LaunchConfiguration('base_seed').perform(context))
    world = create_robot_world(worlds, os.path.join(run_dir, 'terrain_robot_world.sdf'),
                               sensors=sensors == 'true',
                               base_seed=None if base_seed < 0 else base_seed)
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
                        '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                        '/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/world/usgs_utah/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                        '/world/usgs_utah/create@ros_gz_interfaces/srv/SpawnEntity',
                        '/world/usgs_utah/remove@ros_gz_interfaces/srv/DeleteEntity',
                        '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
             remappings=[('/model/robot/odometry', '/model/robot/odometry_raw')],
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='gps_start_publisher',
             name='gps_start_publisher',
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='elevation_map_publisher',
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='lidar_mapper',
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='rviz2', executable='rviz2',
             name='terrain_rviz',
             arguments=['-d', os.path.join(get_package_share_directory('s1_navigation'),
                                          'rviz', 'terrain.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='rviz2', executable='rviz2',
             name='local_mapping_rviz',
             arguments=['-d', os.path.join(get_package_share_directory('s1_navigation'),
                                          'rviz', 'local_mapping.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='gps_odometry',
             name='gps_odometry',
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='gps_waypoint_publisher',
             parameters=[{'use_sim_time': True,
                          'min_distance_from_start': float(LaunchConfiguration('waypoint_start_distance').perform(context)),
                          'min_distance_between_goals': float(LaunchConfiguration('waypoint_spacing').perform(context)),
                          'random_seed': int(LaunchConfiguration('waypoint_seed').perform(context))}],
             prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='gps_waypoint_converter',
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='odom_tf_broadcaster',
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--z', '0.30', '--frame-id', 'base_link',
                        '--child-frame-id', 'robot/base_link/lidar'],
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='astar_planner',
             condition=IfCondition(LaunchConfiguration('follow_path')),
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='path_visualizer',
             condition=IfCondition(LaunchConfiguration('follow_path')),
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
        Node(package='s1_navigation', executable='path_controller',
             condition=IfCondition(LaunchConfiguration('follow_path')),
             parameters=[{'use_sim_time': True}], prefix=prefix, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz with the elevation GridMap display'),
        DeclareLaunchArgument('waypoint_start_distance', default_value='200.0',
                              description='Minimum waypoint distance from GPS origin in metres'),
        DeclareLaunchArgument('waypoint_spacing', default_value='200.0',
                              description='Minimum distance between waypoints in metres'),
        DeclareLaunchArgument('waypoint_seed', default_value='-1',
                              description='Random seed; -1 generates a new set each launch'),
        DeclareLaunchArgument('base_seed', default_value='-1',
                              description='Base layout seed; -1 generates a new layout'),
        DeclareLaunchArgument('gui', default_value='true', description='Open Gazebo viewer'),
        DeclareLaunchArgument('sensors', default_value='true', description='Render the lidar'),
        DeclareLaunchArgument('follow_path', default_value='true',
                              description=(
                                  'Follow /planned_path instead of manual '
                                  'velocity commands')),
        OpaqueFunction(function=launch_terrain),
    ])
