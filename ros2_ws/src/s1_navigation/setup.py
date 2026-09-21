import os
from glob import glob

from setuptools import find_packages, setup


package_name = 's1_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml', 'obstacle_generator.py']
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')
        ),
    ],
    install_requires=['setuptools', 'pyproj'],
    zip_safe=True,
    maintainer='matthew',
    maintainer_email='matthew@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'elevation_map_publisher = s1_navigation.elevation_map_publisher:main',
            'gps_waypoint_publisher = s1_navigation.gps_waypoint_publisher:main',
            'gps_waypoint_converter = s1_navigation.gps_waypoint_converter:main',
            'lidar_mapper = s1_navigation.lidar_mapper:main',
            'odom_tf_broadcaster = s1_navigation.odom_tf_broadcaster:main',
            'waypoint_publisher = s1_navigation.waypoint_publisher:main',
            'astar_planner = s1_navigation.astar_planner:main',
            'path_controller = s1_navigation.path_controller:main',
            'local_planner = s1_navigation.local_planner:main',
            'path_visualizer = s1_navigation.path_visualizer:main',
            'gps_start_publisher = s1_navigation.gps_start_publisher:main',
            'gps_odometry = s1_navigation.gps_odometry:main'
        ],
    },
)
