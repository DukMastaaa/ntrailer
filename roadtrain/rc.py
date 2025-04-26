import socket
import threading
import json

import time


# Set up a separate socket for receiving joystick inputs
JOYSTICK_HOST = ''
JOYSTICK_PORT = 12345
joystick_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
joystick_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

print("Setting up joystick input socket as a TCP server...")

try:
    joystick_sock.bind((JOYSTICK_HOST, JOYSTICK_PORT))
    joystick_sock.listen(10)  # Allow one client connection
    print(f"Joystick input server listening on {JOYSTICK_HOST}:{JOYSTICK_PORT}")
    conn, addr = joystick_sock.accept()
    print(f"Joystick client connected from {addr}")
except Exception as e:
    print(f"Error setting up joystick input server: {e}")
    exit()

joy = None

# Function to handle receiving joystick input data
def receive_joystick_data():
    while True:
        try:
            joystick_data = conn.recv(1024).decode('utf-8')  # Receive data from the client
            if joystick_data:
                print(f"Joystick Input Received: {joystick_data}")
                global joy
                joy = json.loads(joystick_data)
        except Exception as e:
            print(f"Error receiving joystick input data: {e}")
            break

# Start the joystick input receiving thread
joystick_thread = threading.Thread(target=receive_joystick_data, daemon=True).start()



# Set up the socket
HOST = '192.168.4.1'
PORT = 80
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print("connecting...")

sock.connect((HOST, PORT))

print("connected")

# Function to handle receiving data from the socket
def receive_data():
    while True:
        try:
            data = sock.recv(1024).decode('utf-8')
            if data:
                print(f"Received: {data}")
        except Exception as e:
            print(f"Error receiving data: {e}")
            break

# Start the receiving thread
thread = threading.Thread(target=receive_data, daemon=True).start()

try:
    while True:
        if joy is not None:
            left_x = joy["axes"][0]
            left_y = joy["axes"][1]
            data = json.dumps({'left_x': left_x, 'left_y': left_y})
            sock.sendall(data.encode('utf-8'))
        time.sleep(0.1)
        # # Get joystick values
        # x_axis = joystick.get_axis(0)  # Left stick horizontal
        # y_axis = joystick.get_axis(1)  # Left stick vertical
        
        # # Prepare data to send
        # data = json.dumps({'x': x_axis, 'y': y_axis})
        # sock.sendall(data.encode('utf-8'))
        
        # # Print sent data
        # print(f"Sent: {data}")
        
        # pygame.time.wait(100)  # Limit the loop to 10Hz
except KeyboardInterrupt:
    print("Exiting...")
    thread.join()
    joystick_thread.join()
finally:
    sock.close()
    conn.close()
    joystick_sock.close()
    # pygame.quit()