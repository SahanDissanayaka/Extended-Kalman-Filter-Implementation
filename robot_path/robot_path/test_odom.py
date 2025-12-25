# test_odom_pub.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomPub(Node):
    def __init__(self):
        super().__init__('odom_pub')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.x = 0.0

    def timer_cb(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        self.x += 0.01
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdomPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
