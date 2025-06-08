#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import math

class LidController(Node):
    def __init__(self):
        super().__init__('lid_controller')
        
        # Publisher for joint position commands
        self.joint_cmd_pub = self.create_publisher(
            Float64, 
            '/lid_hinge_joint_position_controller/command', 
            10
        )
        
        # Service to control lid
        self.lid_service = self.create_service(
            SetBool, 
            'control_lid', 
            self.control_lid_callback
        )
        
        # Subscriber to monitor joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_lid_angle = 0.0
        self.get_logger().info('Lid Controller Node Started')

    def control_lid_callback(self, request, response):
        """Service callback to open/close lid"""
        try:
            if request.data:  # True = open lid
                target_angle = math.pi / 2  # 90 degrees
                response.message = "Opening lid"
                self.get_logger().info('Opening lid to 90 degrees')
            else:  # False = close lid
                target_angle = 0.0  # 0 degrees
                response.message = "Closing lid"
                self.get_logger().info('Closing lid to 0 degrees')
            
            # Publish target angle
            cmd_msg = Float64()
            cmd_msg.data = target_angle
            self.joint_cmd_pub.publish(cmd_msg)
            
            response.success = True
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error controlling lid: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            return response

    def joint_state_callback(self, msg):
        """Monitor current lid position"""
        try:
            if 'lid_hinge_joint' in msg.name:
                idx = msg.name.index('lid_hinge_joint')
                self.current_lid_angle = msg.position[idx]
        except Exception as e:
            self.get_logger().debug(f'Joint state error: {str(e)}')

    def open_lid(self):
        """Convenience method to open lid"""
        cmd_msg = Float64()
        cmd_msg.data = math.pi / 2
        self.joint_cmd_pub.publish(cmd_msg)
        self.get_logger().info('Lid opened')

    def close_lid(self):
        """Convenience method to close lid"""
        cmd_msg = Float64()
        cmd_msg.data = 0.0
        self.joint_cmd_pub.publish(cmd_msg)
        self.get_logger().info('Lid closed')

def main(args=None):
    rclpy.init(args=args)
    lid_controller = LidController()
    
    try:
        rclpy.spin(lid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        lid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
