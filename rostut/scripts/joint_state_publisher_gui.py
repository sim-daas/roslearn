#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk
from tkinter import Scale

class JointPublisherGUI(Node):
    def __init__(self):
        super().__init__('joint_publisher_gui')
        
        # Publishers
        self.barrel_pub = self.create_publisher(Float64, '/barrel_joint_position_controller/command', 10)
        self.stopper_pub = self.create_publisher(Float64, '/stopper_joint_position_controller/command', 10)
        
        # GUI setup
        self.root = tk.Tk()
        self.root.title("Tank Joint Control")
        
        # Barrel Slider
        self.barrel_slider = Scale(self.root, from_=0.0, to=1.57, resolution=0.01, orient=tk.HORIZONTAL, label="Barrel Angle (rad)", length=400, command=self.publish_barrel)
        self.barrel_slider.pack(pady=10)
        
        # Stopper Slider
        self.stopper_slider = Scale(self.root, from_=0.0, to=0.55, resolution=0.01, orient=tk.HORIZONTAL, label="Stopper Position (m)", length=400, command=self.publish_stopper)
        self.stopper_slider.pack(pady=10)

        # Integrate ROS2 spin with tkinter event loop
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.timer = self.root.after(100, self.ros_spin_once)

    def publish_barrel(self, value):
        msg = Float64()
        msg.data = float(value)
        self.barrel_pub.publish(msg)
        self.get_logger().info(f'Publishing Barrel: {msg.data}')

    def publish_stopper(self, value):
        msg = Float64()
        msg.data = float(value)
        self.stopper_pub.publish(msg)
        self.get_logger().info(f'Publishing Stopper: {msg.data}')

    def ros_spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.timer = self.root.after(100, self.ros_spin_once)

    def on_closing(self):
        """Handles window closing event."""
        self.get_logger().info('Closing GUI and shutting down ROS node.')
        self.root.after_cancel(self.timer)
        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()

    def run_gui(self):
        """Starts the tkinter main loop."""
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    joint_publisher_gui = JointPublisherGUI()
    joint_publisher_gui.run_gui()

if __name__ == '__main__':
    main()
