#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import Scale, Label

class JointPublisherGUI(Node):
    def __init__(self):
        super().__init__('joint_publisher_gui')
        
        # Publishers
        self.barrel_pub = self.create_publisher(Float64, '/barrel_joint_position_controller/command', 10)
        self.stopper_pub = self.create_publisher(Float64, '/stopper_joint_position_controller/command', 10)
        self.rotator_pub = self.create_publisher(Float64, '/rotate_joint_position_controller/command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0 # rad/s

        # GUI setup
        self.root = tk.Tk()
        self.root.title("Tank Joint Control")
        
        # Barrel Slider
        self.barrel_slider = Scale(self.root, from_=0.0, to=1.57, resolution=0.01, orient=tk.HORIZONTAL, label="Barrel Angle (rad)", length=400, command=self.publish_barrel)
        self.barrel_slider.pack(pady=10)
        self.barrel_slider.bind("<Button-1>", self.set_slider_from_click)
        
        # Stopper Slider
        self.stopper_slider = Scale(self.root, from_=0.0, to=0.55, resolution=0.01, orient=tk.HORIZONTAL, label="Stopper Position (m)", length=400, command=self.publish_stopper)
        self.stopper_slider.pack(pady=10)
        self.stopper_slider.bind("<Button-1>", self.set_slider_from_click)

        # Rotator Slider
        self.rotator_slider = Scale(self.root, from_=-3.14, to=3.14, resolution=0.01, orient=tk.HORIZONTAL, label="Rotator Angle (rad)", length=400, command=self.publish_rotator)
        self.rotator_slider.pack(pady=10)
        self.rotator_slider.bind("<Button-1>", self.set_slider_from_click)

        # Keyboard control instructions
        self.key_instructions = Label(self.root, text="Use WASD keys to move the robot.\nClick on the window to focus for keyboard control.")
        self.key_instructions.pack(pady=10)

        # Bind keyboard events
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

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

    def publish_rotator(self, value):
        msg = Float64()
        msg.data = float(value)
        self.rotator_pub.publish(msg)
        self.get_logger().info(f'Publishing Rotator: {msg.data}')

    def on_key_press(self, event):
        key = event.keysym.lower()
        changed = True
        if key == 'w':
            self.linear_vel = self.linear_speed
        elif key == 's':
            self.linear_vel = -self.linear_speed
        elif key == 'a':
            self.angular_vel = self.angular_speed
        elif key == 'd':
            self.angular_vel = -self.angular_speed
        else:
            changed = False
        
        if changed:
            self.publish_cmd_vel()

    def on_key_release(self, event):
        key = event.keysym.lower()
        changed = True
        if key in ['w', 's']:
            self.linear_vel = 0.0
        elif key in ['a', 'd']:
            self.angular_vel = 0.0
        else:
            changed = False
        
        if changed:
            self.publish_cmd_vel()

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Publishing cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def set_slider_from_click(self, event):
        """Moves the slider to the clicked position."""
        widget = event.widget
        if widget.cget('orient') == tk.HORIZONTAL:
            from_ = widget.cget('from')
            to_ = widget.cget('to')
            width = widget.winfo_width()
            
            # get the x-coordinate of the click relative to the widget
            x = event.x
            
            # ensure x is within the widget's bounds
            x = max(0, min(x, width))
            
            # calculate the value
            percent = x / width
            value = from_ + percent * (to_ - from_)
            
            widget.set(value)

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
