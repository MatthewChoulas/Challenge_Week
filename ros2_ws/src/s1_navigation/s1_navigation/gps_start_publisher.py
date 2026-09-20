"""Publish a constant GPS fix for testing the GPS-to-local transform."""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsStartPublisher(Node):

    def __init__(self):
        super().__init__('gps_start_publisher')

        self.latitude = self.declare_parameter(
            'latitude', 38.42287240335025).value
        self.longitude = self.declare_parameter(
            'longitude', -110.78495572815902).value
        self.altitude = self.declare_parameter('altitude', 0.0).value
        publish_rate = self.declare_parameter('publish_rate', 1.0).value
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.publisher = self.create_publisher(
            NavSatFix,
            '/robot_gps_start_location',
            10
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_fix)

        self.get_logger().info(
            f'Publishing GPS fix ({self.latitude:.8f}, '
            f'{self.longitude:.8f}) on /robot_gps_start_location')

    def parameter_callback(self, parameters):
        values = {
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
        }
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value

        if not -90.0 <= values['latitude'] <= 90.0:
            return SetParametersResult(successful=False, reason='latitude must be between -90 and 90')
        if not -180.0 <= values['longitude'] <= 180.0:
            return SetParametersResult(successful=False, reason='longitude must be between -180 and 180')

        self.latitude = values['latitude']
        self.longitude = values['longitude']
        self.altitude = values['altitude']
        self.get_logger().info(
            f'Updated GPS fix to ({self.latitude:.8f}, '
            f'{self.longitude:.8f}, {self.altitude:.2f} m)')
        return SetParametersResult(successful=True)

    def publish_fix(self):
        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'gps'
        message.status.status = message.status.STATUS_FIX
        message.status.service = message.status.SERVICE_GPS
        message.latitude = self.latitude
        message.longitude = self.longitude
        message.altitude = self.altitude
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = GpsStartPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
