import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    share_path = get_package_share_directory("robot_bringup")
    map_dir = os.path.join(share_path, 'config', 'v1.yaml')

    lifecycle_nodes = ['map_server']
    
    namespace = LaunchConfiguration('namespace')

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_dir},
                    {'frame_id': 'map'}],
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld