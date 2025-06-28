#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import subprocess
import math
import os
from ament_index_python.packages import get_package_share_directory

class BallSpawner(Node):
    def __init__(self):
        super().__init__('ball_spawner')
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Store current joint positions
        self.barrel_angle = 0.0  # barrel_joint angle
        self.rotator_angle = 0.0  # rotator_joint angle
        
        # Ball counter for unique naming
        self.ball_counter = 0
        
        # Get package path for sphere SDF
        self.pkg_share = get_package_share_directory('rostut')
        self.sphere_sdf_path = os.path.join(self.pkg_share, 'models', 'sphere.sdf')
        
        # Create service for spawning balls
        self.spawn_service = self.create_service(
            Empty,
            'spawn_ball',
            self.spawn_ball_callback
        )
        
        self.get_logger().info('Ball spawner ready. Call /spawn_ball service to spawn a ball.')

    def joint_state_callback(self, msg):
        """Update joint positions from joint state messages."""
        try:
            # Find barrel_joint and rotator_joint indices
            if 'barrel_joint' in msg.name:
                barrel_idx = msg.name.index('barrel_joint')
                self.barrel_angle = msg.position[barrel_idx]
            
            if 'rotator_joint' in msg.name:
                rotator_idx = msg.name.index('rotator_joint')
                self.rotator_angle = msg.position[rotator_idx]
                
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Joint not found in message: {e}')

    def calculate_ball_position(self):
        """
        Calculate ball spawn position based on barrel and rotator angles.
        This uses trigonometry to position the ball at the tip of the barrel.
        """
        # Base position of the barrel joint (from SDF)
        base_x = 0.5
        base_y = 0.0
        base_z = 0.455
        
        # Barrel length (approximate distance from joint to tip)
        barrel_length = 0.15
        
        # Calculate position considering both rotator and barrel angles
        # Rotator rotates around Z-axis (yaw)
        # Barrel rotates around Y-axis (pitch) but in rotated coordinate system
        
        # Apply rotator rotation (around Z-axis)
        cos_rotator = math.cos(self.rotator_angle)
        sin_rotator = math.sin(self.rotator_angle)
        
        # Apply barrel elevation (around rotated Y-axis)
        cos_barrel = math.cos(self.barrel_angle)
        sin_barrel = math.sin(self.barrel_angle)
        
        # Calculate offset from barrel joint to tip
        # In local coordinates: barrel points in +X direction when at 0 elevation
        local_x = barrel_length * cos_barrel
        local_y = 0.0
        local_z = barrel_length * sin_barrel
        
        # Transform to global coordinates considering rotator rotation
        global_x = base_x + (local_x * cos_rotator - local_y * sin_rotator)
        global_y = base_y + (local_x * sin_rotator + local_y * cos_rotator)
        global_z = base_z + local_z
        
        return global_x, global_y, global_z

    def spawn_ball_callback(self, request, response):
        """Service callback to spawn a ball."""
        try:
            # For now, use fixed coordinates as requested
            # x, y, z = self.calculate_ball_position()  # Uncomment for dynamic positioning
            x, y, z = 0.65, 0.0, 0.455  # Fixed coordinates as requested
            
            # Generate unique ball name
            ball_name = f'ball_{self.ball_counter}'
            self.ball_counter += 1
            
            # Spawn command
            spawn_cmd = [
                'gz', 'service', '-s', '/world/empty/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf_filename: "{self.sphere_sdf_path}", name: "{ball_name}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
            ]
            
            # Execute spawn command
            result = subprocess.run(spawn_cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info(f'Successfully spawned {ball_name} at ({x:.3f}, {y:.3f}, {z:.3f})')
                self.get_logger().info(f'Barrel angle: {self.barrel_angle:.3f}, Rotator angle: {self.rotator_angle:.3f}')
            else:
                self.get_logger().error(f'Failed to spawn ball: {result.stderr}')
                
        except Exception as e:
            self.get_logger().error(f'Error spawning ball: {str(e)}')
            
        return response

def main(args=None):
    rclpy.init(args=args)
    ball_spawner = BallSpawner()
    
    try:
        rclpy.spin(ball_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        ball_spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
