from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Simple lid control node
        Node(
            package='lid_control',
            executable='lid_control_node',
            name='lid_control_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
