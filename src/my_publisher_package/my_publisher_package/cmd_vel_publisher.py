import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32  # Or custom message for wheel speed if desired
import serial  # Requires `pyserial` package

class TeleopSerial(Node):
    def __init__(self):
        super().__init__('teleop_serial')
        self.publisher_left_wheel = self.create_publisher(Float32, 'left_wheel_speed', 10)
        self.publisher_right_wheel = self.create_publisher(Float32, 'right_wheel_speed', 10)

        # Initialize serial connection
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Update with correct port
        self.timer = self.create_timer(0.1, self.read_serial_data)  # Reads at 10 Hz

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            left_speed, right_speed = map(float, data.split(','))  # Assuming data is `left,right`

            # Publish wheel speeds
            left_msg = Float32()
            right_msg = Float32()
            left_msg.data = left_speed
            right_msg.data = right_speed
            self.publisher_left_wheel.publish(left_msg)
            self.publisher_right_wheel.publish(right_msg)
            self.get_logger().info(f"Left: {left_speed} m/s, Right: {right_speed} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

