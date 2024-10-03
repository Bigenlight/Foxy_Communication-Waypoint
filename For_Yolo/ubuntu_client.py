import cv2
import socket
import struct
import pickle

# Server (Windows PC) IP and port
# espers_5g
# server_ip = '192.168.0.107'
# window(ICES lab 5g)
#server_ip = '192.168.0.12' 
# hotspot(phone)
#server_ip = '192.168.151.154' 
# laptop(thedering)
#server_ip = '192.168.137.1'
# 본가 데탑 2.4g
#server_ip = '192.168.35.151' 
# window(ICES lab 5g)
server_ip = '10.50.34.48' 
server_port = 8000

# Create a TCP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_ip, server_port))

# Open a connection to the webcam
cam = cv2.VideoCapture(0)  # Use 0 or the index of your webcam

if not cam.isOpened():
    print("Cannot open camera")
    exit()

try:
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Serialize the frame
        data = pickle.dumps(frame, protocol=pickle.HIGHEST_PROTOCOL)
        # Pack the size of the frame
        message_size = struct.pack("!I", len(data))

        # Send the size and the frame
        client_socket.sendall(message_size + data)

except Exception as e:
    print("Error:", e)

finally:
        cam.release()
        client_socket.close()
