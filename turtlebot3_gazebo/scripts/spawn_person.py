#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import threading
import sys
import os
import random

class PersonSpawner(Node):
    def __init__(self):
        super().__init__('person_spawner')
        
        # Person properties
        self.person_name = 'target_person'
        
        # Model path - use the existing person_standing model
        self.model_path = '/root/devws/src/roslearn/turtlebot3_gazebo/models/person_standing/model.sdf'
        
        self.person_spawned = False
        
        # Check if model file exists
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model file not found: {self.model_path}')
            return
        
        # Initial spawn at random location
        self.spawn_person_random()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info('Person Spawner ready! Commands:')
        self.get_logger().info('  r - Remove person')
        self.get_logger().info('  s - Spawn person at random location')
        self.get_logger().info('  m - Move person to new random location')
        self.get_logger().info('  q - Quit')

    def get_random_position(self):
        """Generate random position within the house world bounds"""
        # House world approximate bounds (avoid walls)
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        z = 0.0
        
        # Avoid spawning too close to the robot's initial position (-2.0, -0.5)
        while abs(x - (-2.0)) < 1.0 and abs(y - (-0.5)) < 1.0:
            x = random.uniform(-3.0, 3.0)
            y = random.uniform(-3.0, 3.0)
        
        return x, y, z

    def spawn_person_random(self):
        """Spawn person at a random location"""
        x, y, z = self.get_random_position()
        self.spawn_person(x, y, z)

    def spawn_person(self, x=0.0, y=0.0, z=0.0):
        """Spawn the person using gz service"""
        if self.person_spawned:
            self.get_logger().warn('Person already spawned!')
            return
            
        try:
            # Use gz service to spawn the model
            cmd = [
                'gz', 'service', '-s', '/world/turtlebot3_house/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'sdf_filename: "{self.model_path}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}, name: "{self.person_name}"'
            ]
            
            self.get_logger().info(f'Spawning person at ({x:.2f}, {y:.2f}, {z:.2f})...')
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.person_spawned = True
                self.get_logger().info(f'Person spawned successfully at ({x:.2f}, {y:.2f})')
            else:
                self.get_logger().error(f'Failed to spawn person: {result.stderr}')
                # Try alternative method using ros2 run
                self.spawn_person_ros2(x, y, z)
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Spawn command timed out')
        except Exception as e:
            self.get_logger().error(f'Error spawning person: {e}')

    def spawn_person_ros2(self, x, y, z):
        """Alternative spawn method using ros2 run"""
        try:
            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', self.model_path,
                '-name', self.person_name,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.person_spawned = True
                self.get_logger().info(f'Person spawned using ros2 method at ({x:.2f}, {y:.2f})')
            else:
                self.get_logger().error(f'Alternative spawn method also failed: {result.stderr}')
                
        except Exception as e:
            self.get_logger().error(f'Alternative spawn error: {e}')

    def remove_person(self):
        """Remove the person using gz service"""
        if not self.person_spawned:
            self.get_logger().warn('No person to remove!')
            return
            
        try:
            cmd = [
                'gz', 'service', '-s', '/world/turtlebot3_house/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'name: "{self.person_name}", type: MODEL'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.person_spawned = False
                self.get_logger().info('Person removed successfully')
            else:
                self.get_logger().error(f'Failed to remove person: {result.stderr}')
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Remove command timed out')
        except Exception as e:
            self.get_logger().error(f'Error removing person: {e}')

    def move_person(self):
        """Move person to a new random location"""
        # Remove current person
        if self.person_spawned:
            self.remove_person()
            
            # Wait a bit for removal to complete
            import time
            time.sleep(1)
        
        # Spawn at new random location
        self.spawn_person_random()

    def input_handler(self):
        """Handle user input in separate thread"""
        while rclpy.ok():
            try:
                command = input().strip().lower()
                
                if command == 'r':
                    self.remove_person()
                elif command == 's':
                    self.spawn_person()
                elif command == 'm':
                    self.move_person()
                elif command == 'q':
                    self.get_logger().info('Shutting down...')
                    rclpy.shutdown()
                    break
                else:
                    self.get_logger().info('Invalid command. Use: r (remove), s (spawn), m (move), q (quit)')
                    
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f'Input error: {e}')

    def __del__(self):
        """Cleanup - no temp files to clean up now"""
        pass

def main(args=None):
    rclpy.init(args=args)
    spawner = PersonSpawner()
    
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        pass
    finally:
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
