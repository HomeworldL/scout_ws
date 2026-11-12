from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='can0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='scout_base', executable='scout_base_node', name='scout_base',
             output='screen', parameters=[PathJoinSubstitution([
                 FindPackageShare('lunar_hardware_bringup'), 'config', 'scout.yaml']), {
                     'port_name': LaunchConfiguration('port_name'),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
             # Keep wheel velocity feedback, without a second parent for base_link.
             remappings=[('/tf', '/scout/wheel_tf')]),
    ])
