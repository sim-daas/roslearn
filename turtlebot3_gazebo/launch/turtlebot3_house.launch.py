#!/usr/bin/env python3

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Generate timestamp for unique map filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    maps_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'maps')
    
    # Create maps directory if it doesn't exist
    os.makedirs(maps_dir, exist_ok=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    mode = LaunchConfiguration('mode', default='slam')  # slam or navigation

    # Declare launch arguments
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Mode: slam for mapping, navigation for navigation with existing map'
    )

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time        }.items()
    )

    # RViz2 node for visualization with custom config
    rviz_config_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        'tb3_slam.rviz'
    )
    
    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # SLAM Toolbox using online async launch from slam_toolbox package
    slam_config_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': slam_config_file
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),                'models'))

    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_mode_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(rviz2_cmd)
    ld.add_action(slam_toolbox_cmd)

    return ld
