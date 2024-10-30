# lidar_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.declare_parameter('lidar_topic', '/scan')
        self.publisher_ = self.create_publisher(LaserScan, self.get_parameter('lidar_topic').value, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        msg.angle_min = 0.0
        msg.angle_max = 6.28
        msg.angle_increment = 0.1
        msg.time_increment = 0.0
        msg.scan_time = 1.0
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [1.0] * 63
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

