#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class LidTester(Node):
    def __init__(self):
        super().__init__('lid_tester')
        
        # Wait for service to be available
        self.lid_client = self.create_client(SetBool, 'control_lid')
        while not self.lid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lid control service...')

    def test_lid_sequence(self):
        """Test opening and closing the lid"""
        
        # Open lid
        self.get_logger().info('Testing lid opening...')
        request = SetBool.Request()
        request.data = True
        
        future = self.lid_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f'Success: {future.result().message}')
        else:
            self.get_logger().error(f'Failed: {future.result().message}')
        
        # Wait 3 seconds
        time.sleep(3)
        
        # Close lid
        self.get_logger().info('Testing lid closing...')
        request.data = False
        
        future = self.lid_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f'Success: {future.result().message}')
        else:
            self.get_logger().error(f'Failed: {future.result().message}')

def main(args=None):
    rclpy.init(args=args)
    tester = LidTester()
    
    try:
        tester.test_lid_sequence()
    except Exception as e:
        tester.get_logger().error(f'Test failed: {str(e)}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
