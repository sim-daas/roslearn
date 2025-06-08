#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64
import math

class LidControlNode(Node):
    def __init__(self):
        super().__init__('lid_control_node')
        
        # Publisher for lid joint position
        self.lid_pub = self.create_publisher(Float64, '/lid_joint_position_controller/command', 10)
        
        # Service for controlling lid
        self.control_service = self.create_service(SetBool, 'control_lid', self.control_lid_callback)
        
        self.get_logger().info('Lid Control Node initialized')
        
        # Lid positions (in radians)
        self.lid_closed_pos = 0.0
        self.lid_open_pos = math.pi / 2  # 90 degrees
        
    def control_lid_callback(self, request, response):
        """Service callback to open/close lid"""
        try:
            msg = Float64()
            if request.data:  # Open lid
                msg.data = self.lid_open_pos
                self.get_logger().info('Opening lid')
            else:  # Close lid
                msg.data = self.lid_closed_pos
                self.get_logger().info('Closing lid')
                
            self.lid_pub.publish(msg)
            response.success = True
            response.message = f"Lid {'opened' if request.data else 'closed'} successfully"
            
        except Exception as e:
            self.get_logger().error(f'Failed to control lid: {str(e)}')
            response.success = False
            response.message = f"Failed to control lid: {str(e)}"
            
        return response

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
