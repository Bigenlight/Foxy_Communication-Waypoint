import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class CoordinatePublisher(Node):

    def __init__(self):
        super().__init__('coordinate_publisher')

        self.publisher_ = self.create_publisher(String, 'gps_location', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_coordinates)
        self.lat = 37.640641
        self.lng = 127.093191

    def publish_coordinates(self):
        msg = String()
        msg.data = f"({self.lat}, {self.lng})"
        self.publisher_.publish(msg)
        self.lat += 0.0001
        self.lng += 0.0001
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    coordinate_publisher = CoordinatePublisher()
    rclpy.spin(coordinate_publisher)
    coordinate_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
