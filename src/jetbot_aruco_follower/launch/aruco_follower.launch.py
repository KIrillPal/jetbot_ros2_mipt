"""Launch the HTTP bridge, central detector, and disabled follower."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the teleop-compatible ArUco follower stack."""
    base_url = LaunchConfiguration('camera_server_url')
    config_file = LaunchConfiguration('config_file')
    focal_length = LaunchConfiguration('camera_focal_length_px')
    camera_info_file = LaunchConfiguration('camera_info_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_server_url',
            default_value='http://127.0.0.1:8080',
            description='Base URL of the host camera server',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetbot_aruco_follower'),
                'config',
                'aruco_follower.yaml',
            ]),
            description='ROS 2 parameter YAML for detector and follower',
        ),
        DeclareLaunchArgument(
            'camera_info_file',
            default_value='',
            description='ROS calibration YAML published as camera info',
        ),
        DeclareLaunchArgument(
            'camera_focal_length_px',
            default_value='0.0',
            description='Measured focal length in pixels; zero uses camera FOV',
        ),
        Node(
            package='jetbot_aruco_follower',
            executable='http_camera_bridge',
            name='aruco_http_camera_bridge',
            output='screen',
            parameters=[{
                'base_url': base_url,
                'camera_info_file': camera_info_file,
            }],
        ),
        Node(
            package='jetbot_aruco_follower',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[config_file, {
                'camera_focal_length_px': ParameterValue(
                    focal_length, value_type=float),
            }],
        ),
        Node(
            package='jetbot_aruco_follower',
            executable='aruco_follower',
            name='aruco_follower',
            output='screen',
            parameters=[config_file],
        ),
    ])
