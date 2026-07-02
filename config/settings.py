import numpy as np

# Network
ESP8266_IP = "10.90.17.34"
ESP8266_PORT = 5005

# Vision & Grid
GRID_SIZE = 20
OBSTACLE_CLASS_ID = 0
PIXELS_PER_CM = 3

# Color Tracking (Pink)
LOWER_PINK = np.array([130, 50, 50])
UPPER_PINK = np.array([180, 255, 255])

# Wiggle Settings
MOVE_DURATION = 0.3
STOP_DURATION = 0.5
POSSIBLE_COMMANDS = ['W', 'A', 'S', 'D']
SEQUENCE_LENGTH = 5