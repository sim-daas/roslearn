#!/usr/bin/env python3

import numpy as np
from PIL import Image
import os

def create_test_map():
    # Create a simple test map (400x400 pixels)
    map_data = np.full((400, 400), 205, dtype=np.uint8)  # Free space (gray)

    # Add some walls (black boundaries)
    map_data[0:10, :] = 0      # Top wall
    map_data[-10:, :] = 0      # Bottom wall  
    map_data[:, 0:10] = 0      # Left wall
    map_data[:, -10:] = 0      # Right wall

    # Add some internal obstacles
    map_data[100:120, 100:300] = 0  # Horizontal wall
    map_data[200:350, 200:220] = 0  # Vertical wall

    # Save as PGM
    maps_dir = '/root/devws/src/roslearn/turtlebot3_gazebo/maps'
    os.makedirs(maps_dir, exist_ok=True)
    
    img = Image.fromarray(map_data, mode='L')
    img.save(os.path.join(maps_dir, 'map_latest.pgm'))
    print(f'Test map created at {os.path.join(maps_dir, "map_latest.pgm")}')

if __name__ == '__main__':
    create_test_map()