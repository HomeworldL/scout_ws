from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('lunar_hardware_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('pcd', description='Absolute path to the saved map.pcd'),
        DeclareLaunchArgument('base_z', default_value='0.18108',
                              description='Initial base_link height in map, in metres'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_yaw', default_value='1.5707963267948966'),
        DeclareLaunchArgument('lidar_config', default_value=PathJoinSubstitution([
            share, 'config', 'mid360.json'])),
        DeclareLaunchArgument('lio_config', default_value=PathJoinSubstitution([
            FindPackageShare('fastlio2'), 'config', 'lio.yaml'])),
        DeclareLaunchArgument('localizer_config', default_value=PathJoinSubstitution([
            FindPackageShare('localizer'), 'config', 'localizer.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'lidar.launch.py'])),
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            launch_arguments={
                'lidar_config': LaunchConfiguration('lidar_config'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
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
        Node(package='localizer', executable='localizer_node', namespace='localizer',
             name='localizer_node', output='screen', parameters=[{
                 'config_path': LaunchConfiguration('localizer_config'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        # Display the saved map before the first /initialpose, without triggering ICP.
        Node(package='pcl_ros', executable='pcd_to_pointcloud', name='saved_map_cloud',
             output='screen', parameters=[{
                 'file_name': LaunchConfiguration('pcd'), 'tf_frame': 'map',
                 'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('cloud_pcd', '/localization/reference_cloud')]),
        Node(package='lunar_hardware_bringup', executable='initial_pose_bridge.py',
             name='initial_pose_bridge', output='screen', parameters=[{
                 'pcd_path': LaunchConfiguration('pcd'),
                 'base_z': ParameterValue(LaunchConfiguration('base_z'), value_type=float),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'localization.rviz'])],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
