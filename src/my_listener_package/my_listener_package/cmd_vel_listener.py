import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import struct
import math
import tf_transformations
import time

class CmdVelListener(Node):
    def __init__(self, wheel_base):
        super().__init__('cmd_vel_listener')

        # Parameters
        self.wheel_base = wheel_base
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Create a subscriber to listen to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Configure the serial connection to ESP32
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # Update with the actual port
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            exit(1)

        # Time tracking for odometry calculation
        self.prev_time = time.time()

    def listener_callback(self, msg):
        """Callback to handle incoming Twist messages and send them to ESP32."""
        linear = msg.linear.x
        angular = msg.angular.z

        # Send the packed data over serial
        try:
            buff = struct.pack('=BBff', 36, 36, linear, angular)
            self.ser.write(buff)
            self.get_logger().info(f"Sent packed data: {buff}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send packed data: {e}")

        # Calculate odometry
        self.calculate_odometry(linear, angular)

    def calculate_odometry(self, linear, angular):
        """Calculate and publish odometry based on linear and angular velocity."""
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Update position and orientation
        delta_x = linear * math.cos(self.theta) * dt
        delta_y = linear * math.sin(self.theta) * dt
        delta_theta = angular * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create quaternion from theta
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion(*quaternion)

        # Velocity
        odom_msg.twist.twist.linear.x = linear
        odom_msg.twist.twist.angular.z = angular

        # Publish odometry
        self.odom_publisher.publish(odom_msg)
        self.get_logger().info(f"Published odometry: x={self.x}, y={self.y}, theta={self.theta}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener(wheel_base=0.5)  # Replace with your actual wheel base
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

