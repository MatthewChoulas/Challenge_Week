#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/robot/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info('Odometry TF broadcaster started')


    def odom_callback(self, msg):

        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp

        # Use exactly what Gazebo publishes
        transform.header.frame_id = msg.header.frame_id
        transform.child_frame_id = msg.child_frame_id

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):

    rclpy.init(args=args)

    node = OdomTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    # ROS may already have shut the default context down after SIGINT.
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
