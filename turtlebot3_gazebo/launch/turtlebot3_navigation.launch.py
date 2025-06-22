#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Maps directory
    maps_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'maps')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    map_file = LaunchConfiguration('map', default=os.path.join(maps_dir, 'map_latest.yaml'))

    # Declare launch arguments
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(maps_dir, 'map_latest.yaml'),
        description='Full path to map yaml file to load')

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
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time
        }.items()
    )

    # RViz2 node for visualization with navigation config (delayed start)
    rviz_config_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        'tb3_navigation.rviz'
    )
    
    # Check if RViz config exists, fallback to default if not
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'rviz',
            'tb3_slam.rviz'
        )
    
    rviz2_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # Nav2 navigation configuration
    nav2_config_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'config',
        'nav2_params.yaml'
    )
    
    # Map server and localization (start first)
    localization_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'localization_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_config_file,
                    'map': map_file
                }.items()
            )
        ]
    )
    
    # Nav2 navigation launch (start after localization)
    nav2_cmd = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_config_file,
                    'use_composition': 'True'
                }.items()
            )
        ]
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))

    ld = LaunchDescription()
    
    # Add the commands to the launch description in proper order
    ld.add_action(declare_map_arg)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz2_cmd)

    return ld