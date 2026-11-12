from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('lunar_hardware_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_yaw', default_value='1.5707963267948966'),
        DeclareLaunchArgument('lio_config', default_value=PathJoinSubstitution([
            FindPackageShare('fastlio2'), 'config', 'lio.yaml'])),
        DeclareLaunchArgument('pgo_config', default_value=PathJoinSubstitution([
            FindPackageShare('pgo'), 'config', 'pgo.yaml'])),
        # Keep the description's rviz=false local to this include.
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('scout_mini_description'), 'launch', 'description.launch.py'])),
                launch_arguments={'rviz': 'false',
                                  'lidar_yaw': LaunchConfiguration('lidar_yaw'),
                                  'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        ]),
        Node(package='fastlio2', executable='lio_node', namespace='fastlio2',
             name='lio_node', output='screen', parameters=[{
                 'config_path': LaunchConfiguration('lio_config'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='pgo', executable='pgo_node', namespace='pgo',
             name='pgo_node', output='screen', parameters=[{
                 'config_path': LaunchConfiguration('pgo_config'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'mapping.rviz'])],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
