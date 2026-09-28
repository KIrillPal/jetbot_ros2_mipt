#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='Lidar communication channel type',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyLIDAR',
            description='Lidar serial device',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Lidar serial baud rate',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='lidar_link',
            description='Frame ID used for laser scans',
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Whether to invert scan data',
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Whether to enable angle compensation',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Standard',
            description='Lidar scan mode',
        ),
    ]

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': LaunchConfiguration('channel_type'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': LaunchConfiguration('inverted'),
            'angle_compensate': LaunchConfiguration('angle_compensate'),
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
        output='screen',
    )

    return LaunchDescription(declared_arguments + [lidar])
