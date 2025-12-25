#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')

        # State: [x, y, theta]
        self.x = np.zeros((3, 1))

        # Covariance
        self.P = np.eye(3) * 0.1

        # Noise matrices
        self.Q = np.diag([0.05, 0.05, 0.02])   # process noise
        self.R = np.diag([0.1, 0.1, 0.05])    # measurement noise

        self.last_time = self.get_clock().now()

        self.cmd_vel = np.zeros((2, 1))

        # Subscribers
        self.create_subscription(Twist, '/diff_drive_base_controller/cmd_vel', self.cmd_callback, 10)
        #self.create_subscription(Odometry, '/diff_drive_base_controller/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/noisy_odom', self.odom_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)
        self.path_pub = self.create_publisher(Path, '/ekf_path', 10)

        # Initialize path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        # Timer for predict step
        self.timer = self.create_timer(0.02, self.predict)  # 50 Hz

        self.get_logger().info('EKF Node started')

    def cmd_callback(self, msg: Twist):
        self.cmd_vel[0, 0] = msg.linear.x
        self.cmd_vel[1, 0] = msg.angular.z

    def predict(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        v = self.cmd_vel[0, 0]
        w = self.cmd_vel[1, 0]
        theta = self.x[2, 0]

        # State prediction
        self.x[0, 0] += v * np.cos(theta) * dt
        self.x[1, 0] += v * np.sin(theta) * dt
        self.x[2, 0] += w * dt

        # Jacobian F
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0,  1]
        ])

        self.P = F @ self.P @ F.T + self.Q

    def odom_callback(self, msg: Odometry):

        z = np.zeros((3, 1))
        z[0, 0] = msg.pose.pose.position.x
        z[1, 0] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, z[2, 0] = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Measurement model H
        H = np.eye(3)

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        self.publish_pose()

    def publish_pose(self):

        # Publish PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.position.x = float(self.x[0, 0])
        msg.pose.position.y = float(self.x[1, 0])
        q = quaternion_from_euler(0, 0, self.x[2, 0])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pose_pub.publish(msg)

        # Append to Path and publish
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(msg)
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
