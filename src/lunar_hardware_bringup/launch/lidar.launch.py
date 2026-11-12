from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_config', default_value=PathJoinSubstitution([
            FindPackageShare('lunar_hardware_bringup'), 'config', 'mid360.json'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen', parameters=[{
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'user_config_path': LaunchConfiguration('lidar_config'),
                 'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                 'publish_freq': 10.0, 'output_data_type': 0,
                 'frame_id': 'livox_frame', 'lvx_file_path': '',
                 'cmdline_input_bd_code': 'livox0000000001',
             }]),
    ])
