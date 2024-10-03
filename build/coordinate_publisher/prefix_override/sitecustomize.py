import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dj/Foxy_Communication-Waypoint/install/coordinate_publisher'
