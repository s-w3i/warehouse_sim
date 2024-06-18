import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import ReplaceString


def generate_launch_description():

    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='AGV1',
        description='Top-level namespace')
    
    arg_show_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="start RViz automatically with the launch file",
    )
    
    arg_namespace = DeclareLaunchArgument(
        "use_namespace",
        default_value="true",
        description="whether using namespace or not",
    )
    
    declare_frame_prefix = DeclareLaunchArgument(
        'frame_prefix',
        default_value='AGV1/',
        description='Frame Prefix for tf tree'
    )

    namespace = LaunchConfiguration('namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')

    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("robot_bringup"),
            "urdf",
            "diffbot_system.urdf.xacro"
        ]),
        " ",
        "prefix:=", frame_prefix,  # Correct usage of LaunchConfiguration in Command
        " ",
        "robot_name:=", namespace
    ])
    
    robot_description = {"robot_description": robot_description_content}
     
    node_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            namespace=namespace,
    )

    return LaunchDescription(
        [
            arg_show_rviz,
            arg_namespace,
            declare_namespace_cmd,
            declare_frame_prefix,
            node_robot_state_publisher,
        ]
    )
