# client_test.py
import socket

server_ip = '192.168.0.107'  # Windows PC IP
server_port = 8000

try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print("Connected to server")
except Exception as e:
    print(f"Connection failed: {e}")
finally:
    client_socket.close()
