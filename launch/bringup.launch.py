"""Launch the package nodes with shared ROS parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Resolve params from the installed package share directory.
    default_params_file = PathJoinSubstitution([
        FindPackageShare('ff_magpie_ros'),
        'config',
        'params.yaml',
    ])
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('ff_magpie_ros'),
        'config',
        'slam_toolbox.yaml',
    ])
    slam_launch_file = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    ])

    # LaunchDescription lists the actions ROS runs for this launch.
    return LaunchDescription([
        # Allow a different parameter YAML to be selected, e.g.
        # params_file:=/path/to/params.yaml on the ros2 launch command.
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the ROS parameter file.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Path to the slam_toolbox parameter file.',
        ),
        # Nodes
        Node(
            package='ff_magpie_ros',
            executable='ethernet_reader',
            name='ethernet_reader',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='ff_magpie_ros',
            executable='ld06_dtof_lidar_parser',
            name='ld06_dtof_lidar_parser',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='ff_magpie_ros',
            executable='data_preplanner',
            name='data_preplanner',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        # Use slam_toolbox's lifecycle-aware async mapping launch.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
