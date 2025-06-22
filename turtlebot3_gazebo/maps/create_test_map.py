# This is a placeholder - you should replace this with an actual saved map
# For now, we'll create a simple shell script to generate a basic test map

# Run this to create a basic test map:
# python3 -c "
import numpy as np
from PIL import Image
import os

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
img = Image.fromarray(map_data, mode='L')
img.save('/root/devws/src/roslearn/turtlebot3_gazebo/maps/map_latest.pgm')
print('Test map created at /root/devws/src/roslearn/turtlebot3_gazebo/maps/map_latest.pgm')
"