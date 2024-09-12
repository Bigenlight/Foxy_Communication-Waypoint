import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MQTTSubscriber(Node):

    def __init__(self):
        super().__init__('mqtt_subscriber')

        # Create publishers
        self.coord_publisher = self.create_publisher(String, 'coordination_list', 10)
        self.state_publisher = self.create_publisher(String, 'motor_state', 10)

        # Create subscriptions
        self.subscription = self.create_subscription(
            String,
            'coordination_list',
            self.listener_callback,
            10)
        self.gps_subscription = self.create_subscription(
            String,
            'gps_location',
            self.gps_listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.gps_subscription  # prevent unused variable warning

        # MQTT client setup
        self.mqtt_client = mqtt.Client("UbuntuSubscriber", userdata=None, protocol=mqtt.MQTTv311, transport="tcp")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to MQTT broker
        # window(ICES lab 5g)
        #self.mqtt_client.connect("192.168.0.12", 1883, 60)  
        # hotspot(phone)
        #self.mqtt_client.connect("192.168.151.154", 1883, 60)
        # laptop(thedering)
        self.mqtt_client.connect("192.168.137.1", 1883, 60)

        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            client.subscribe("test/topic")
            client.subscribe("motor_state")  # Subscribe to motor_state topic
        else:
            self.get_logger().error(f"Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"Received message: {msg.payload.decode()}, topic: {msg.topic}")
        ros_msg = String()
        ros_msg.data = msg.payload.decode()

        # Check the topic and publish to the appropriate ROS2 topic
        if msg.topic == "test/topic":
            self.coord_publisher.publish(ros_msg)
        elif msg.topic == "motor_state":
            self.state_publisher.publish(ros_msg)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

    def gps_listener_callback(self, msg):
        self.get_logger().info(f'Received GPS location: "{msg.data}"')
        # Send GPS location via MQTT
        self.mqtt_client.publish("gps_location_topic", msg.data)

def main(args=None):
    rclpy.init(args=args)

    mqtt_subscriber = MQTTSubscriber()

    rclpy.spin(mqtt_subscriber)

    mqtt_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
