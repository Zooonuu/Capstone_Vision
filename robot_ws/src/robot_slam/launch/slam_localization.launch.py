"""Localization-mode launch (this is the one full_pipeline.launch.py uses):
static TF + odom-instance EKF + map-instance EKF + slam_toolbox (localization).

Usage:
    ros2 launch robot_slam slam_localization.launch.py map_file_name:=/home/<user>/maps/room
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_slam')
    map_file_name = LaunchConfiguration('map_file_name')

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'static_tf.launch.py'])
        )
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'ekf.launch.py'])
        ),
        launch_arguments={'mode': 'localization'}.items(),
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_localization.yaml']),
            {'map_file_name': ParameterValue(map_file_name, value_type=str)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file_name', default_value='',
            description='Path prefix (no extension) of a map serialized by '
                        'slam_toolbox in mapping mode, e.g. /home/user/maps/room'),
        static_tf,
        ekf,
        slam_toolbox_node,
    ])
