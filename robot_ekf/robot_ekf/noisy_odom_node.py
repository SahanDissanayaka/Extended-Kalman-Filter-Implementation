#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class NoisyOdom(Node):
    def __init__(self):
        super().__init__('noisy_odom_node')

        self.sub = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',  # ground truth
            self.callback,
            10
        )

        self.pub = self.create_publisher(Odometry, '/noisy_odom', 10)

        self.noise_pos = 0.05
        self.noise_yaw = 0.02

    def callback(self, msg: Odometry):
        noisy = Odometry()
        noisy.header = msg.header
        noisy.child_frame_id = msg.child_frame_id

        # Position noise
        noisy.pose.pose.position.x = msg.pose.pose.position.x + np.random.normal(0, self.noise_pos)
        noisy.pose.pose.position.y = msg.pose.pose.position.y + np.random.normal(0, self.noise_pos)

        # Orientation noise
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw += np.random.normal(0, self.noise_yaw)

        qn = quaternion_from_euler(0, 0, yaw)
        noisy.pose.pose.orientation.x = qn[0]
        noisy.pose.pose.orientation.y = qn[1]
        noisy.pose.pose.orientation.z = qn[2]
        noisy.pose.pose.orientation.w = qn[3]

        noisy.twist = msg.twist
        self.pub.publish(noisy)

def main():
    rclpy.init()
    node = NoisyOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
