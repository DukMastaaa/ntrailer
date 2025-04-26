import pygame

# Initialize Pygame and the joystick module
pygame.init()
pygame.joystick.init()

print("Joysticks:", pygame.joystick.get_count())

try:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick initialized: {joystick.get_name()}")
except pygame.error:
    print("No joystick found.")
    exit()

# Main loop to read joystick input
try:
    while True:
        pygame.event.pump()  # Process events
        
        # Get joystick axes (e.g., left stick horizontal and vertical)
        x_axis = joystick.get_axis(0)  # Left stick horizontal
        y_axis = joystick.get_axis(1)  # Left stick vertical
        
        # Get button states
        button_0 = joystick.get_button(0)  # Example: Cross button on PlayStation
        
        # Print joystick values
        print(f"X Axis: {x_axis}, Y Axis: {y_axis}, Button 0: {button_0}")
        
        pygame.time.wait(100)  # Limit the loop to 10Hz
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pygame.quit()