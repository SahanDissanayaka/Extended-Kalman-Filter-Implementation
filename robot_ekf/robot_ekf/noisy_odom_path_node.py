#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class NoisyOdomPath(Node):
    def __init__(self):
        super().__init__('noisy_odom_path')
        self.sub = self.create_subscription(Odometry, '/noisy_odom', self.callback, 10)
        self.pub = self.create_publisher(Path, '/noisy_path', 10)
        self.path = Path()
        self.path.header.frame_id = "odom"

    def callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)
        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = NoisyOdomPath()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
