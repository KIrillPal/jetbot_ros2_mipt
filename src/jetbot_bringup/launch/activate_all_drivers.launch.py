from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[PathJoinSubstitution([
            FindPackageShare('jetbot_bringup'),
            'config',
            'twist_mux.yaml',
        ])],
        remappings=[
            ('/cmd_vel_out', '/diffbot_base_controller/cmd_vel_unstamped'),
        ],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('jetbot_bringup'),
                'launch',
                'rplidar.launch.py',
            ])
        )
    )

    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('diffdrive_jetbot'),
                'launch',
                'diffbot.launch.py',
            ])
        )
    )

    return LaunchDescription([
        twist_mux,
        lidar_launch,
        diffbot_launch,
    ])
