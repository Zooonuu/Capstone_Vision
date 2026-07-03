"""robot_localization EKF nodes.

mode:=mapping    -> only the odom-instance EKF (odom -> base_link). slam_toolbox
                    itself owns map -> odom in mapping mode.
mode:=localization -> both the odom-instance and map-instance EKF (the latter
                    fuses slam_toolbox's /pose to publish map -> odom).
See config/ekf_odom.yaml and config/ekf_map.yaml for the fusion details.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    pkg_share = get_package_share_directory('robot_slam')
    is_localization = PythonExpression(["'", mode, "' == 'localization'"])

    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf_odom.yaml'])],
    )

    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf_map.yaml'])],
        condition=IfCondition(is_localization),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='mapping | localization; controls whether the map-instance '
                        'EKF (which needs slam_toolbox /pose) is started.'),
        ekf_odom_node,
        ekf_map_node,
    ])
