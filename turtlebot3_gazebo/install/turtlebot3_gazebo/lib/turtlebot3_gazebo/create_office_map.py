#!/usr/bin/env python3

import numpy as np
import os
import sys

def create_office_map():
    """
    Create a simple office map as text (since PIL might not be available)
    White (255) = free space
    Black (0) = occupied space  
    Gray (205) = unknown space
    """
    
    # Map dimensions in pixels (20m x 15m at 0.05m/pixel = 400x300 pixels)
    width = 400  # 20m / 0.05m
    height = 300  # 15m / 0.05m
    
    # Initialize map as free space (white)
    map_data = np.full((height, width), 255, dtype=np.uint8)
    
    # Add outer walls (black = occupied)
    # North wall (top)
    map_data[0:4, :] = 0
    # South wall (bottom) 
    map_data[-4:, :] = 0
    # East wall (right)
    map_data[:, -4:] = 0
    # West wall (left)
    map_data[:, 0:4] = 0
    
    # Add internal walls
    # Central corridor wall 1 (x=-2m, y from -1m to 7m)
    x_pixel = int((8 * 20) / 400)  # x=-2m -> pixel 160
    y_start = int((6.5 * 15) / 300)  # y=-1m -> pixel 65
    y_end = int((14.5 * 15) / 300)   # y=7m -> pixel 290
    map_data[y_start:y_end, x_pixel-2:x_pixel+2] = 0
    
    # Central corridor wall 2 (x=2m, y from -7m to 1m)
    x_pixel = int((12 * 20) / 400)  # x=2m -> pixel 240
    y_start = int((0.5 * 15) / 300)  # y=-7m -> pixel 10
    y_end = int((8.5 * 15) / 300)    # y=1m -> pixel 170
    map_data[y_start:y_end, x_pixel-2:x_pixel+2] = 0
    
    # Add furniture as obstacles
    # Reception desk (-8, 0) -> pixels (40, 150)
    desk_x = int((2 * 20) / 400)
    desk_y = int((7.5 * 15) / 300)
    desk_w = int((2 * 20) / 400)  # 2m wide
    desk_h = int((1 * 15) / 300)  # 1m deep
    map_data[desk_y-desk_h//2:desk_y+desk_h//2, desk_x-desk_w//2:desk_x+desk_w//2] = 0
    
    # Office chairs and desks
    # Chair 1 (-6, 5) -> pixels (80, 225)
    chair_x = int((4 * 20) / 400)
    chair_y = int((12.5 * 15) / 300)
    chair_size = int((0.6 * 20) / 400)
    map_data[chair_y-chair_size//2:chair_y+chair_size//2, chair_x-chair_size//2:chair_x+chair_size//2] = 0
    
    # Desk 1 (-6, 5.5) -> pixels (80, 235)
    desk1_x = int((4 * 20) / 400)
    desk1_y = int((13 * 15) / 300)
    desk1_w = int((1.5 * 20) / 400)
    desk1_h = int((0.8 * 15) / 300)
    map_data[desk1_y-desk1_h//2:desk1_y+desk1_h//2, desk1_x-desk1_w//2:desk1_x+desk1_w//2] = 0
    
    # Chair 2 (6, -5) -> pixels (320, 75)
    chair2_x = int((16 * 20) / 400)
    chair2_y = int((2.5 * 15) / 300)
    map_data[chair2_y-chair_size//2:chair2_y+chair_size//2, chair2_x-chair_size//2:chair2_x+chair_size//2] = 0
    
    # Desk 2 (6, -5.5) -> pixels (320, 65)
    desk2_x = int((16 * 20) / 400)
    desk2_y = int((2 * 15) / 300)
    map_data[desk2_y-desk1_h//2:desk2_y+desk1_h//2, desk2_x-desk1_w//2:desk2_x+desk1_w//2] = 0
    
    # Meeting table (5, 3) -> pixels (300, 195)
    table_x = int((15 * 20) / 400)
    table_y = int((10.5 * 15) / 300)
    table_w = int((3 * 20) / 400)
    table_h = int((1.5 * 15) / 300)
    map_data[table_y-table_h//2:table_y+table_h//2, table_x-table_w//2:table_x+table_w//2] = 0
    
    # Filing cabinet (8, 6) -> pixels (360, 255)
    cabinet_x = int((18 * 20) / 400)
    cabinet_y = int((13.5 * 15) / 300)
    cabinet_w = int((0.6 * 20) / 400)
    cabinet_h = int((0.4 * 15) / 300)
    map_data[cabinet_y-cabinet_h//2:cabinet_y+cabinet_h//2, cabinet_x-cabinet_w//2:cabinet_x+cabinet_w//2] = 0
    
    # Plant (-8, 3) -> pixels (40, 195)
    plant_x = int((2 * 20) / 400)
    plant_y = int((10.5 * 15) / 300)
    plant_size = int((0.6 * 20) / 400)
    map_data[plant_y-plant_size//2:plant_y+plant_size//2, plant_x-plant_size//2:plant_x+plant_size//2] = 0
    
    return map_data

def save_pgm(data, filename):
    """Save data as PGM format"""
    height, width = data.shape
    with open(filename, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(data.tobytes())

def main():
    print("Creating office map...")
    
    # Create the map
    office_map = create_office_map()
    
    # Create maps directory if it doesn't exist
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(os.path.dirname(script_dir), 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    
    # Save as PGM file
    pgm_path = os.path.join(maps_dir, 'office_map.pgm')
    
    try:
        # Try to use PIL if available
        from PIL import Image
        img = Image.fromarray(office_map, mode='L')
        img.save(pgm_path)
        print(f"Office map created using PIL: {pgm_path}")
    except ImportError:
        # Fallback to manual PGM writing
        save_pgm(office_map, pgm_path)
        print(f"Office map created manually: {pgm_path}")
    
    print(f"Map dimensions: {office_map.shape[1]} x {office_map.shape[0]} pixels")
    print("Map coordinate system:")
    print("  Origin: (-10.0, -7.5, 0.0)")
    print("  Resolution: 0.05 m/pixel")
    print("  Size: 20m x 15m")

if __name__ == '__main__':
    main()
