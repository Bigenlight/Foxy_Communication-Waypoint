import pandas as pd
from pyproj import Proj, transform

# Define the reference GPS coordinates (latitude and longitude)
ref_lat = 37.640607  # Latitude of the reference point (origin)
ref_lon = 127.091603  # Longitude of the reference point (origin)

# Define the GPS waypoints to convert (latitude, longitude)
gps_waypoints = [
    (37.640607, 127.091603),  # Start point (origin)
    (37.644175, 127.093534),  # Point 1
    (37.638738, 127.094779),  # Point 2
    (37.64285, 127.095165)    # Point 3
]

# Define a projection from WGS84 (GPS coordinates) to a local ENU coordinate system
proj_wgs84 = Proj(proj='latlong', datum='WGS84')
proj_enu = Proj(proj='tmerc', lat_0=ref_lat, lon_0=ref_lon, k=1, x_0=0, y_0=0, datum='WGS84')

# Convert GPS coordinates to local ENU coordinates
relative_waypoints = []
for lat, lon in gps_waypoints:
    x, y = transform(proj_wgs84, proj_enu, lon, lat)  # Note: order is (lon, lat) for pyproj
    relative_waypoints.append((x, y))

# Prepare data for the CSV file
data = {
    'X-axis': [x for x, y in relative_waypoints],
    'Y-axis': [y for x, y in relative_waypoints],
    'Link': [0, 1, 2, 3]  # Assign Link numbers for each waypoint
}

# Save the data to a CSV file
df = pd.DataFrame(data)
df.to_csv('relative_waypoints.csv', index=False)

print("Relative waypoints saved to 'relative_waypoints.csv'.")
