import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/theo/Foxy_Communication-Waypoint/install/mqtt_subscriber'
