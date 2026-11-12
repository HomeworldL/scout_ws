from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("scout_mini_description")
    description = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([share, "urdf", "scout_mini.urdf.xacro"]),
        " lidar_yaw:=", LaunchConfiguration("lidar_yaw"),
    ])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lidar_yaw", default_value="1.5707963267948966"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(description, value_type=str),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }], output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", PathJoinSubstitution([share, "rviz", "description.rviz"])],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("rviz")), output="screen",
        ),
    ])
