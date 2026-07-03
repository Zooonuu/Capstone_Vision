"""Mapping-mode launch: static TF + odom-instance EKF + slam_toolbox (mapping).

Usage:
    ros2 launch robot_slam slam_mapping.launch.py
    # drive the robot around, then save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/room
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
        "{filename: '/home/<user>/maps/room'}"
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_slam')

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'static_tf.launch.py'])
        )
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'ekf.launch.py'])
        ),
        launch_arguments={'mode': 'mapping'}.items(),
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_mapping.yaml'])],
    )

    return LaunchDescription([
        static_tf,
        ekf,
        slam_toolbox_node,
    ])
ROS2 팀원은 이 점수를