import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class EKFPathNode(Node):
    def __init__(self):
        super().__init__('ekf_path_node')

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.sub = self.create_subscription(
            PoseStamped,
            '/ekf_pose',
            self.pose_callback,
            10
        )

        self.pub = self.create_publisher(Path, '/ekf_path', 10)

    def pose_callback(self, msg):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(msg)
        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = EKFPathNode()
    rclpy.spin(node)
    rclpy.shutdown()
