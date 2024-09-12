#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
from pyproj import Proj, Transformer

class WaypointConverter(Node):
    def __init__(self):
        super().__init__('waypoint_converter')
        self.subscription = self.create_subscription(
            String,
            'coordination_list',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Waypoint Converter Node has been started.')

    def listener_callback(self, msg):
        gps_data = msg.data.strip()

        # Parse the GPS waypoints from the message
        gps_waypoints = self.parse_gps_data(gps_data)
        if not gps_waypoints:
            self.get_logger().error('No valid GPS waypoints found in the message.')
            return

        # Use the first GPS point as the reference (origin) for relative conversion
        ref_lat, ref_lon = gps_waypoints[0][:2]

        # Define a projection from WGS84 (GPS coordinates) to a local ENU coordinate system
        transformer = Transformer.from_crs("EPSG:4326", f"+proj=tmerc +lat_0={ref_lat} +lon_0={ref_lon} +k=1 +x_0=0 +y_0=0 +datum=WGS84", always_xy=True)

        # Convert GPS coordinates to local ENU coordinates
        relative_waypoints = []
        for lat, lon, link in gps_waypoints:
            x, y = transformer.transform(lon, lat)
            relative_waypoints.append((x, y, link))

        # Save the converted waypoints to a CSV file
        self.save_to_csv(relative_waypoints)
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
        df.to_csv('relative_waypoints.csv', index=False)


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
