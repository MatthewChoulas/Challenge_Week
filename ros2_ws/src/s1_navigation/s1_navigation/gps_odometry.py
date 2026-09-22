"""Move Gazebo to a GPS start position and relay its actual odometry."""

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty

from .utm_conversion import latlon_to_local


class GpsOdometry(Node):

    def __init__(self):
        super().__init__('gps_odometry')

        self.origin_latitude = self.declare_parameter(
            'origin_latitude', 38.42287240335025).value
        self.origin_longitude = self.declare_parameter(
            'origin_longitude', -110.78495572815902).value
        self.origin_altitude = self.declare_parameter(
            'origin_altitude', 0.0).value
        self.raw_topic = self.declare_parameter(
            'raw_topic', '/model/robot/odometry_raw').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/model/robot/odometry').value
        self.world_name = self.declare_parameter('world_name', 'usgs_utah').value
        self.robot_name = self.declare_parameter('robot_name', 'robot').value

        self.raw_odometry = None
        self.pending_position = None
        self.pose_future = None
        self.last_gps = None

        self.gps_subscription = self.create_subscription(
            NavSatFix, '/robot_gps_start_location', self.gps_callback, 10)
        self.origin_reset_subscription = self.create_subscription(
            Empty, '/reset_rover_to_origin', self.origin_reset_callback, 10)
        self.raw_subscription = self.create_subscription(
            Odometry, self.raw_topic, self.raw_odom_callback, 10)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.pose_client = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')

        self.reset_timer = self.create_timer(0.5, self.reset_gazebo_pose)

        self.get_logger().info(
            f'Resetting {self.raw_topic} with GPS and publishing {self.output_topic}')

    def gps_callback(self, message):
        if (not math.isfinite(message.latitude) or
                not math.isfinite(message.longitude) or
                not math.isfinite(message.altitude)):
            self.get_logger().warning('Ignoring GPS fix containing NaN or infinity')
            return
        if not (-90.0 <= message.latitude <= 90.0 and
                -180.0 <= message.longitude <= 180.0):
            self.get_logger().warning('Ignoring GPS coordinates outside valid ranges')
            return

        gps_fix = (message.latitude, message.longitude, message.altitude)
        # The start publisher repeats its fix. Only a new coordinate should
        # trigger another reset, otherwise the rover would be reset every
        # timer cycle while it is moving.
        if gps_fix == self.last_gps:
            return
        self.last_gps = gps_fix

        east, north = latlon_to_local(
            message.latitude, message.longitude,
            self.origin_latitude, self.origin_longitude)
        gps_local = (
            east,
            north,
            message.altitude - self.origin_altitude,
        )

        # Keep the newest target queued until the simulator is ready.
        self.pending_position = gps_local

    def origin_reset_callback(self, _message):
        """Queue an explicit origin reset even if the GPS fix is unchanged."""
        self.pending_position = (0.0, 0.0, 0.0)
        self.last_gps = (
            self.origin_latitude, self.origin_longitude, self.origin_altitude)

    def reset_gazebo_pose(self):
        if self.pending_position is None or self.raw_odometry is None:
            return
        if self.pose_future is not None or not self.pose_client.service_is_ready():
            return

        target = self.pending_position
        raw_pose = self.raw_odometry.pose.pose
        request = SetEntityPose.Request()
        request.entity.name = self.robot_name
        request.entity.type = request.entity.MODEL
        request.pose.position.x = target[0]
        request.pose.position.y = target[1]
        # TerrainDrive determines the height and tilt at the new XY position.
        request.pose.position.z = raw_pose.position.z
        request.pose.orientation = copy.deepcopy(raw_pose.orientation)
        self.pose_future = self.pose_client.call_async(request)
        self.pose_future.add_done_callback(
            lambda future: self.pose_reset_done(future, target))

    def pose_reset_done(self, future, target):
        self.pose_future = None
        try:
            success = future.result().success
        except Exception as error:
            self.get_logger().warning(f'Gazebo reset failed; will retry: {error}')
            return
        if not success:
            self.get_logger().warning('Gazebo rejected the reset; will retry')
            return
        if self.pending_position == target:
            self.pending_position = None
        self.get_logger().info(
            f'Gazebo accepted GPS reset to east={target[0]:.3f}, '
            f'north={target[1]:.3f} m; relaying actual simulator odometry')

    def raw_odom_callback(self, message):
        self.raw_odometry = message
        corrected = copy.deepcopy(message)
        # Preserve Gazebo's frame names so the existing odom -> base_link TF
        # remains visible in RViz. The reset changes the pose, not the frame
        # tree convention used by the simulator.
        corrected.header.frame_id = message.header.frame_id or 'odom'
        corrected.child_frame_id = message.child_frame_id or 'base_link'

        # Gazebo is authoritative: do not add the GPS displacement twice.
        self.publisher.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
