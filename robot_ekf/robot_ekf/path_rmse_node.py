#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np

class PathRMSE(Node):
    def __init__(self):
        super().__init__('path_rmse_node')

        self.gt_path = None
        self.noisy_path = None
        self.ekf_path = None

        self.create_subscription(Path, '/odom_path', self.gt_cb, 10)
        self.create_subscription(Path, '/noisy_path', self.noisy_cb, 10)
        self.create_subscription(Path, '/ekf_path', self.ekf_cb, 10)

        self.timer = self.create_timer(2.0, self.compute_rmse)

        self.get_logger().info('Path RMSE node started')

    def gt_cb(self, msg): self.gt_path = msg
    def noisy_cb(self, msg): self.noisy_path = msg
    def ekf_cb(self, msg): self.ekf_path = msg

    def compute_rmse(self):
        if not (self.gt_path and self.noisy_path and self.ekf_path):
            return

        n = min(
            len(self.gt_path.poses),
            len(self.noisy_path.poses),
            len(self.ekf_path.poses)
        )

        gt = np.array([[p.pose.position.x, p.pose.position.y]
                       for p in self.gt_path.poses[:n]])

        noisy = np.array([[p.pose.position.x, p.pose.position.y]
                          for p in self.noisy_path.poses[:n]])

        ekf = np.array([[p.pose.position.x, p.pose.position.y]
                        for p in self.ekf_path.poses[:n]])

        rmse_noisy = np.sqrt(np.mean(np.sum((noisy - gt) ** 2, axis=1)))
        rmse_ekf = np.sqrt(np.mean(np.sum((ekf - gt) ** 2, axis=1)))

        self.get_logger().info(
            f'RMSE Noisy vs GT = {rmse_noisy:.4f} m | '
            f'RMSE EKF vs GT = {rmse_ekf:.4f} m | '
            f'N = {n}'
        )

def main():
    rclpy.init()
    node = PathRMSE()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
