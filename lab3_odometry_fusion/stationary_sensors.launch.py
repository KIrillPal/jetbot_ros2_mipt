"""Temporary stationary commissioning: no mux/teleop/follower; base starts inactive."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = {'robot_description': Command([
        FindExecutable(name='xacro'), ' ', PathJoinSubstitution([
            FindPackageShare('diffdrive_jetbot'), 'urdf', 'diffbot.urdf.xacro']),
        ' robot_ip:=', LaunchConfiguration('robot_ip'), ' mesh_port:=8000'])}
    config = PathJoinSubstitution([FindPackageShare('diffdrive_jetbot'), 'config',
                                  'diffbot_controllers.yaml'])
    joints = Node(package='controller_manager', executable='spawner',
                  arguments=['joint_state_broadcaster', '-c', '/controller_manager'])
    base = Node(package='controller_manager', executable='spawner',
                arguments=['diffbot_base_controller', '-c', '/controller_manager', '--inactive'])
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='10.192.173.147'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[description, config], output='screen',
             remappings=[('/diffbot_base_controller/cmd_vel_unstamped',
                          '/lab3/disabled_motor_commands'),
                         ('/diffbot_base_controller/cmd_vel', '/lab3/disabled_stamped_commands')]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], output='screen'),
        joints,
        RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=[base])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('jetbot_bringup'), 'launch', 'rplidar.launch.py'])),
            launch_arguments={'scan_mode': 'Standard'}.items()),
    ])
