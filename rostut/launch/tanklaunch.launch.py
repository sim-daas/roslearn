import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='rostut').find('rostut')
    
    # Path to the URDF xacro file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # World file path
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of robot'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='Y position of robot'
    )
    
    # Set GZ_RESOURCE_PATH to include models directory
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models')
    )
    
    # Launch GZ Sim server with world
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )
    
    # Launch GZ Sim GUI
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 30.0
        }]
    )
    
    # Spawn robot in Gazebo using SDF file
    sdf_file = os.path.join(pkg_share, 'models', 'tank.sdf')
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'myrobot',
            '-file', sdf_file,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Spawn sphere inside barrel
    sphere_sdf_file = os.path.join(pkg_share, 'models', 'sphere.sdf')
    spawn_sphere_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'barrel_sphere',
            '-file', sphere_sdf_file,
            '-x', '0.65',  # Inside barrel position
            '-y', '0.0',
            '-z', '0.455'
        ],
        output='screen'
    )
    
    # Ball Spawner Node
    ball_spawner_cmd = Node(
        package='rostut',
        executable='ball_spawner',
        name='ball_spawner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ROS-GZ Bridge
    gz_bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_config
        }],
        output='screen'
    )
    
    # RViz2 for visualization
    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_file': os.path.join(pkg_share, 'config', 'robot.rviz')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        x_pose_arg,
        y_pose_arg,
        set_env_vars_resources,
        robot_state_publisher_node,
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_cmd,
        spawn_sphere_cmd,
        ball_spawner_cmd,
        bridge_cmd,
        rviz2_cmd,
    ])
