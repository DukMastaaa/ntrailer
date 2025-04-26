import pygame
import socket
import json
import time

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Set up the TCP client socket
HOST = 'localhost'  # Replace with the desired server host
PORT = 12345         # Replace with the desired server port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print("Connecting to the TCP server...")

try:
    sock.connect((HOST, PORT))
    print("Connected to the TCP server")
except Exception as e:
    print(f"Error connecting to the TCP server: {e}")
    exit()

def get_joystick_data(joystick):
    joystick_data = {
        'axes': [joystick.get_axis(a) for a in range(joystick.get_numaxes())],
        'buttons': [joystick.get_button(b) for b in range(joystick.get_numbuttons())],
        'hats': [joystick.get_hat(h) for h in range(joystick.get_numhats())]
    }
    return joystick_data

def main():
    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        while True:
            pygame.event.pump()  # Process event queue
            data = get_joystick_data(joystick)
            print(data)
            serialized_data = json.dumps(data)
            sock.sendall(serialized_data.encode('utf-8'))  # Send data to the server
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()
        sock.close()

if __name__ == "__main__":
    main()