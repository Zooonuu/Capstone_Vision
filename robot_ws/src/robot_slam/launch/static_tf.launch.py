"""Static TF publishers for the sensor mounts.

Defines: base_link -> laser_link, base_link -> camera_link,
camera_link -> camera_optical_frame.

Defaults below are placeholders (no real hardware measurements yet) and are
documented in robot_slam/README.md. Override them at launch time once the
robot is measured, e.g.:

    ros2 launch robot_slam static_tf.launch.py camera_mount_height:=0.40 camera_tilt_deg:=25.0

NOTE: these numbers only affect the *visual/TF-tree* placement of the frames.
The actual bbox-pixel -> ground-point math used for object localization lives
in robot_perception's ground_projection_node and reads its own
config/camera_extrinsics.yaml (h, theta, focal length) directly, per the
"lidar = obstacle avoidance, camera = target detection, TF = the only place
they meet" separation described in the top-level README. Keep both configs in
sync when the real hardware is measured.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    laser_mount_height = LaunchConfiguration('laser_mount_height')
    camera_mount_height = LaunchConfiguration('camera_mount_height')
    camera_forward_offset = LaunchConfiguration('camera_forward_offset')
    camera_tilt_deg = LaunchConfiguration('camera_tilt_deg')

    # tf2_ros static_transform_publisher expects roll/pitch/yaw in radians on
    # Humble (no --angle-units flag yet), so convert the human-friendly degree
    # argument here rather than relying on distro-specific CLI features.
    camera_tilt_rad = PythonExpression(['3.14159265358979 * ', camera_tilt_deg, ' / 180.0'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'laser_mount_height', default_value='0.05',
            description='Height (m) of laser_link above base_link origin. '
                        'Placeholder: "wheel top + a bit" per hardware assumption.'),
        DeclareLaunchArgument(
            'camera_mount_height', default_value='0.35',
            description='Height (m) of camera_link above base_link origin (robot top).'),
        DeclareLaunchArgument(
            'camera_forward_offset', default_value='0.05',
            description='Forward (x) offset (m) of camera_link from base_link origin.'),
        DeclareLaunchArgument(
            'camera_tilt_deg', default_value='30.0',
            description='Downward tilt angle (deg) of the camera from horizontal, '
                        'so the ground ahead of the robot is visible.'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', laser_mount_height,
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', camera_forward_offset, '--y', '0.0', '--z', camera_mount_height,
                # pitch sign is a placeholder; verify against the real mount and
                # flip if the robot's convention tilts the other way.
                '--roll', '0.0', '--pitch', camera_tilt_rad, '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
        ),
        # REP-103 (x-fwd,y-left,z-up) -> optical convention (x-right,y-down,z-fwd)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
                '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame',
            ],
        ),
    ])
