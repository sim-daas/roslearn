from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Load lid joint controller
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'lid_joint_position_controller'],
            output='screen'
        ),
        
        # Lid control node
        Node(
            package='lid_control',
            executable='lid_control_node',
            name='lid_control_node',
            output='screen'
        ),
        
        # Lid detector node
        Node(
            package='lid_control',
            executable='lid_detector',
            name='lid_detector',
            output='screen'
        ),
    ])
