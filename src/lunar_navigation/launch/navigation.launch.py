from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('lunar_navigation')
    default_map = str(Path(__file__).resolve().parents[3] / 'maps/site/map.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='2D YAML; defaults to workspace/maps/site/map.yaml'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            share, 'config', 'nav2.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{
                 'yaml_filename': LaunchConfiguration('map'),
                 'frame_id': 'map', 'topic_name': 'map',
                 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen', parameters=[{
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'autostart': LaunchConfiguration('autostart'), 'node_names': ['map_server']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': 'False',
            }.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'navigation.rviz'])],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
