from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def prepare_paths(context):
    pcd = Path(LaunchConfiguration('pcd').perform(context)).expanduser().resolve()
    if not pcd.is_file():
        raise RuntimeError(f'PCD map does not exist: {pcd}')
    output = Path(LaunchConfiguration('output').perform(context)).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    return [SetLaunchConfiguration('pcd', str(pcd)),
            SetLaunchConfiguration('output', str(output))]


def generate_launch_description():
    share = FindPackageShare('lunar_navigation')
    site = Path(__file__).resolve().parents[3] / 'maps/site'
    return LaunchDescription([
        DeclareLaunchArgument('pcd', default_value=str(site / 'map.pcd')),
        DeclareLaunchArgument('output', default_value=str(site / 'map'),
                              description='Output prefix, without .pgm/.yaml'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            share, 'config', 'pcd2pgm.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=prepare_paths),
        Node(package='pcd2pgm', executable='pcd2pgm_node', name='pcd2pgm',
             output='screen', parameters=[LaunchConfiguration('params_file'), {
                 'pcd_file': LaunchConfiguration('pcd'),
                 'map_topic_name': '/pcd2pgm/map'}],
             remappings=[('pcd_cloud', '/pcd2pgm/cloud')]),
        # The upstream node publishes OccupancyGrid; Nav2 handles PGM + YAML export.
        Node(package='nav2_map_server', executable='map_saver_cli',
             name='pcd2pgm_map_saver', output='screen',
             arguments=['-t', '/pcd2pgm/map', '-f', LaunchConfiguration('output'),
                        '--fmt', 'pgm', '--mode', 'trinary',
                        '--occ', '0.65', '--free', '0.196'],
             parameters=[{'map_subscribe_transient_local': True, 'save_map_timeout': 30.0}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'pcd2pgm.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
