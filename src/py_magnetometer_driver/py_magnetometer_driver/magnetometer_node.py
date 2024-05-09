import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from PiicoDev_QMC6310 import PiicoDev_QMC6310
from PiicoDev_Unified import sleep_ms

class MagnetometerNode(Node):
    def __init__(self):
        super().__init__('magnetometer_node')
        self.publisher_ = self.create_publisher(MagneticField, 'magnetometer', 10)
        self.mag_sensor = PiicoDev_QMC6310()
        self.mag_sensor.setRange(1200)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer as needed

    def timer_callback(self):
        raw_data = self.mag_sensor.read()
        mag_msg = MagneticField()
        mag_msg.magnetic_field = Vector3(x=raw_data['x'], y=raw_data['y'], z=raw_data['z'])
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'magnetometer_link'  # Update as per your TF configuration
        self.publisher_.publish(mag_msg)
        sleep_ms(100)  # Regulate the loop speed as needed

    def on_shutdown(self):
        self.get_logger().info('Node is shutting down...')
        # Clean up code here, stop timers, release resources, etc.

def main(args=None):
    rclpy.init(args=args)
    magnetometer_node = MagnetometerNode()
    
    rclpy.on_shutdown(magnetometer_node.on_shutdown)
    try:
        rclpy.spin(magnetometer_node)
    except KeyboardInterrupt:
        magnetometer_node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        magnetometer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
