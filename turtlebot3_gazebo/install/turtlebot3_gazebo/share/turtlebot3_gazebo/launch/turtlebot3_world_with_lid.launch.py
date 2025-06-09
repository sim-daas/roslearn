import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # Use the existing TurtleBot3 world launch but with modified robot
    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Lid control node
    lid_control_node = Node(
        package='lid_control',
        executable='lid_control_node',
        name='lid_control_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'x_pose', default_value='-2.0',
            description='x position'),

        DeclareLaunchArgument(
            'y_pose', default_value='-0.5',
            description='y position'),

        turtlebot3_world,
        lid_control_node,
    ])
