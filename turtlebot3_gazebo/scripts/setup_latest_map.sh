#!/bin/bash

# Script to copy the latest map to map_latest for navigation
MAPS_DIR="/root/devws/src/roslearn/turtlebot3_gazebo/maps"

# Find the latest map file
LATEST_MAP=$(ls -t "$MAPS_DIR"/map_*.yaml 2>/dev/null | head -n1)

if [ -n "$LATEST_MAP" ]; then
    # Get the base name without extension
    BASE_NAME=$(basename "$LATEST_MAP" .yaml)
    
    # Copy both yaml and pgm files
    cp "$MAPS_DIR/$BASE_NAME.yaml" "$MAPS_DIR/map_latest.yaml"
    cp "$MAPS_DIR/$BASE_NAME.pgm" "$MAPS_DIR/map_latest.pgm"
    
    # Update the yaml file to reference the correct pgm file
    sed -i "s|$BASE_NAME.pgm|map_latest.pgm|g" "$MAPS_DIR/map_latest.yaml"
    
    echo "Latest map copied to map_latest.yaml and map_latest.pgm"
    echo "Source: $LATEST_MAP"
else
    echo "No map files found in $MAPS_DIR"
    echo "Please create a map first using SLAM mode:"
    echo "ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py"
fi