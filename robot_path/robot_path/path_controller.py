import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time

class PathController(Node):
    def __init__(self):
        super().__init__('path_controller')
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        # Fill header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # Set velocity
        msg.twist.linear.x = 0.3   # m/s
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.3  # rad/s
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PathController()
    rclpy.spin(node)
    rclpy.shutdown()
