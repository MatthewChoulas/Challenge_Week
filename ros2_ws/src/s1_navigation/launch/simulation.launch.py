import os
import subprocess
import sys
import tempfile
import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def launch_simulation(context):

    s1_share = get_package_share_directory(
        's1_navigation'
    )

    world_file = os.path.join(
        s1_share,
        'worlds',
        's1_world.sdf'
    )


    regenerate = LaunchConfiguration('regenerate_obstacles').perform(context).lower()
    if regenerate not in ('true', 'false'):
        raise ValueError('regenerate_obstacles must be true or false')
    if regenerate == 'true':
        # Each launch gets its own world, so another generation cannot change
        # the file used by this run's waypoint publisher.
        run_dir = tempfile.mkdtemp(prefix='s1_navigation_')
        subprocess.run([
            sys.executable, os.path.join(s1_share, 'obstacle_generator.py'),
            '--template', os.path.join(s1_share, 'worlds', 's1_world_template.sdf'),
            '--output-dir', run_dir,
        ], check=True)
        world_file = os.path.join(run_dir, 's1_world.sdf')

    # No shell between launch and the supervisor. Separate GUI/server processes
    # let closing the GUI explicitly shut down the server and navigation nodes.
    supervisor = [sys.executable, '-m', 's1_navigation.managed_process']
    native_gui = os.path.join(
        get_package_prefix('s1_planar_drive'), 'bin', 's1_gazebo_gui')
    gazebo_server = ExecuteProcess(
        cmd=supervisor + ['gz', 'sim', '-s', '-r', world_file, '--force-version', '8'],
        name='gazebo_server', output='screen',
        on_exit=Shutdown(reason='Gazebo server exited'),
        sigterm_timeout='5', sigkill_timeout='5',
    )
    gazebo_gui = ExecuteProcess(
        cmd=supervisor + [native_gui],
        name='gazebo_gui', output='screen',
        on_exit=Shutdown(reason='Gazebo window closed'),
        sigterm_timeout='5', sigkill_timeout='5',
    )
    node_prefix = f'{sys.executable} -m s1_navigation.managed_process'

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',

        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Gazebo -> ROS
            '/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            # Gazebo -> ROS
            '/model/robot/odometry'
            '@nav_msgs/msg/Odometry'
            '[gz.msgs.Odometry',

            # ROS -> Gazebo
            '/model/robot/cmd_vel'
            '@geometry_msgs/msg/Twist'
            ']gz.msgs.Twist',
        ],

        parameters=[{'use_sim_time': True}],
        prefix=node_prefix,
        output='screen'
    )


    odom_tf = Node(
        package='s1_navigation',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        parameters=[{'use_sim_time': True}],
        prefix=node_prefix,
        output='screen'
    )


    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform',

        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.30',

            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',

            '--frame-id', 'base_link',
            '--child-frame-id', 'robot/base_link/lidar',
        ],

        parameters=[{'use_sim_time': True}],
        prefix=node_prefix,
        output='screen'
    )


    lidar_mapper = Node(
        package='s1_navigation',
        executable='lidar_mapper',
        name='lidar_mapper',
        parameters=[{'use_sim_time': True}],
        prefix=node_prefix,
        output='screen'
    )


    waypoint_publisher = Node(
        package='s1_navigation',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        parameters=[{'use_sim_time': True, 'world_file': world_file}],
        prefix=node_prefix,
        output='screen'
    )


    astar_planner = Node(
        package='s1_navigation',
        executable='astar_planner',
        name='astar_planner',
        parameters=[{'use_sim_time': True}],
        prefix=node_prefix,
        output='screen'
    )


    path_controller = Node(
        package='s1_navigation',
        executable='path_controller',
        name='path_controller',
        parameters=[{'use_sim_time': True}],
        prefix=node_prefix,
        output='screen'
    )

    return [
        LogInfo(msg=f"Simulation and waypoints use world: {world_file}"),
        # Keep this GUI and bridge attached to this launch's server only.
        SetEnvironmentVariable(
            name="GZ_PARTITION", value=f"s1_navigation_{uuid.uuid4().hex}"),
        gazebo_server,
        gazebo_gui,
        bridge,

        lidar_tf,
        odom_tf,

        lidar_mapper,
        waypoint_publisher,

        astar_planner,
        path_controller,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'regenerate_obstacles', default_value='true',
            description='Generate a fresh obstacle world before starting the simulation',
        ),
        OpaqueFunction(function=launch_simulation),
    ])
