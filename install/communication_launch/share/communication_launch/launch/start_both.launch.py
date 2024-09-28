from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_converter',
            executable='waypoint_converter',  # Updated executable name
            name='waypoint_converter'
        ),
        Node(
            package='mqtt_subscriber',
            executable='mqtt_subscriber',  # Updated executable name
            name='mqtt_subscriber'
        )
    ])
