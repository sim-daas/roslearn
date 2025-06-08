#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64
import math

class LidControlNode(Node):
    def __init__(self):
        super().__init__('lid_control_node')
        
        # Publisher for lid joint position (matching SDF topic exactly)
        self.lid_pub = self.create_publisher(Float64, '/lid_joint_position_controller/command', 10)
        
        # Service for controlling lid
        self.control_service = self.create_service(SetBool, 'control_lid', self.control_lid_callback)
        
        # Timer to ensure connection is established
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.connection_established = False
        
        self.get_logger().info('Lid Control Node initialized')
        
        # Lid positions (in radians)
        self.lid_closed_pos = 0.0
        self.lid_open_pos = math.pi / 2  # 90 degrees
        
    def timer_callback(self):
        """Check if publisher is connected"""
        subscription_count = self.lid_pub.get_subscription_count()
        if not self.connection_established and subscription_count > 0:
            self.connection_established = True
            self.get_logger().info(f'Lid controller connected! Subscribers: {subscription_count}')
        elif self.connection_established and subscription_count == 0:
            self.connection_established = False
            self.get_logger().warn('Lid controller disconnected!')
        
    def control_lid_callback(self, request, response):
        """Service callback to open/close lid"""
        try:
            msg = Float64()
            if request.data:  # Open lid
                msg.data = self.lid_open_pos
                self.get_logger().info('Opening lid to 90 degrees')
                response.message = "Lid opened"
            else:  # Close lid
                msg.data = self.lid_closed_pos
                self.get_logger().info('Closing lid to 0 degrees')
                response.message = "Lid closed"
                
            # Publish command multiple times to ensure it's received
            for i in range(10):
                self.lid_pub.publish(msg)
                self.get_logger().debug(f'Published lid command {i+1}/10: {msg.data}')
                
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Failed to control lid: {str(e)}')
            response.success = False
            response.message = f"Failed to control lid: {str(e)}"
            
        return response
    
    def open_lid_direct(self):
        """Direct method to open lid"""
        msg = Float64()
        msg.data = self.lid_open_pos
        for _ in range(5):
            self.lid_pub.publish(msg)
        self.get_logger().info('Lid opened directly')
        
    def close_lid_direct(self):
        """Direct method to close lid"""
        msg = Float64()
        msg.data = self.lid_closed_pos
        for _ in range(5):
            self.lid_pub.publish(msg)
        self.get_logger().info('Lid closed directly')

def main(args=None):
    rclpy.init(args=args)
    node = LidControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
