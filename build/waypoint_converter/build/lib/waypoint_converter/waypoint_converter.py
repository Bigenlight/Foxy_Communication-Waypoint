#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
from pyproj import Transformer

class WaypointConverter(Node):
    def __init__(self):
        super().__init__('waypoint_converter')
        
        # Initialize the reference coordinates
        self.ref_lat = None
        self.ref_lon = None
        
        # Subscriber for the GPS location topic
        self.create_subscription(
            String,
            'gps_location',
            self.gps_callback,
            10
        )

        # Subscriber for the coordination list topic
        self.create_subscription(
            String,
            'coordination_list',
            self.listener_callback,
            10
        )

        self.get_logger().info('Waypoint Converter Node has been started.')

    def gps_callback(self, msg):
        """
        Callback to handle the incoming GPS location messages
        to set the reference latitude and longitude.
        """
        try:
            # Parse the GPS location from the message
            gps_data = msg.data.strip().strip(' ').strip(';').strip('()')
            self.ref_lat, self.ref_lon = map(float, gps_data.split(','))

            self.get_logger().info(f'Set reference GPS location to: ({self.ref_lat}, {self.ref_lon})')
        except Exception as e:
            self.get_logger().error(f'Error parsing GPS location: {e}')

    def listener_callback(self, msg):
        """
        Callback to handle the incoming coordination list messages.
        """
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().warn('Reference GPS location is not set yet. Waiting for GPS data...')
            return

        gps_data = msg.data.strip()

        # Parse the GPS waypoints from the message
        gps_waypoints = self.parse_gps_data(gps_data)
        if not gps_waypoints:
            self.get_logger().error('No valid GPS waypoints found in the message.')
            return

        # Use the first GPS point as the reference (origin) for relative conversion
        transformer = Transformer.from_crs(
            "EPSG:4326",
            f"+proj=tmerc +lat_0={self.ref_lat} +lon_0={self.ref_lon} +k=1 +x_0=0 +y_0=0 +datum=WGS84",
            always_xy=True
        )

        # Convert GPS coordinates to local ENU coordinates
        relative_waypoints = []
        for lat, lon, link in gps_waypoints:
            x, y = transformer.transform(lon, lat)
            relative_waypoints.append((x, y, link))
        
        # **Add interpolation between waypoints**
        dense_waypoints = self.interpolate_waypoints(relative_waypoints, 50) # 좌표 50배

        # Save the converted waypoints to a CSV file
        self.save_to_csv(dense_waypoints)
        self.get_logger().info("Relative waypoints saved to 'relative_waypoints.csv'.")

    def parse_gps_data(self, data):
        """
        Parses the GPS waypoint string and extracts (latitude, longitude, link) tuples.
        """
        waypoints = []
        try:
            # Remove 'start;' and 'end;' from the message
            data = data.replace('start;', '').replace('end;', '').strip()
            points = data.split(';')

            for point in points:
                point = point.strip()
                if point:
                    # Extract the GPS coordinates and link number
                    lat, lon, link = point.strip('()').split(',')
                    waypoints.append((float(lat), float(lon), int(link)))
        except Exception as e:
            self.get_logger().error(f"Error parsing GPS data: {e}")
            return []

        return waypoints

    def interpolate_waypoints(self, waypoints, num_points_between):
        """
        Adds interpolated waypoints between each pair of waypoints.
        """
        dense_waypoints = []
        for i in range(len(waypoints) - 1):
            x1, y1, link1 = waypoints[i]
            x2, y2, link2 = waypoints[i + 1]
            # Interpolate between waypoints
            for j in range(num_points_between + 1):
                t = j / (num_points_between + 1)
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                link = link1  # Use the link of the starting waypoint
                dense_waypoints.append((x, y, link))
        # Add the last waypoint
        dense_waypoints.append(waypoints[-1])
        return dense_waypoints

    def save_to_csv(self, waypoints):
        """
        Saves the converted waypoints to a CSV file.
        """
        data = {
            'X-axis': [x for x, y, link in waypoints],
            'Y-axis': [y for x, y, link in waypoints],
            'Link': [link for x, y, link in waypoints]
        }

        df = pd.DataFrame(data)

        # Save to CSV file (update the path as needed)
        df.to_csv('../2023CAPSTONE_AutoCar_in_Ros2/AutoCarROS2/autocar_map/data/relative_waypoints.csv', index=False)

def main(args=None):
    rclpy.init(args=args)
    waypoint_converter = WaypointConverter()

    try:
        rclpy.spin(waypoint_converter)
    except KeyboardInterrupt:
        waypoint_converter.get_logger().info('Waypoint Converter Node stopped.')
    finally:
        waypoint_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
