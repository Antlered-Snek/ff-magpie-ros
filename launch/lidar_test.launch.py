"""Launch the LD06 parser and RViz configured to display its scan."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ff_magpie_ros'),
        'rviz',
        'lidar_test.rviz',
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
        # Nodes
        Node(
            package='ff_magpie_ros',
            executable='ld06_dtof_lidar_parser',
            name='ld06_dtof_lidar_parser',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        # Show /scan in a 2D top-down view, using the parser's laser_frame.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
