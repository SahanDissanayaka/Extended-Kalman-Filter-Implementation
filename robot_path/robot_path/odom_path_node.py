#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathNode(Node):

    def __init__(self):
        super().__init__('odom_path_node')

        # Subscribe to odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publish path
        self.path_pub = self.create_publisher(
            Path,
            '/odom_path',
            10
        )

        # Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.get_logger().info('Odom Path Node started')

    def odom_callback(self, msg: Odometry):

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append pose to path
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)

        # Publish path
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
