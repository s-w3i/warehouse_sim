import os
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap, Node

def generate_launch_description():

    fleet_server = Node(
        package='free_fleet_server_ros2',
        executable='free_fleet_server_ros2',
        name='fleet_server_node',
        output='both',
        parameters=[
            {'fleet_name': 'v1'},
            {'fleet_state_topic': 'fleet_states'},
            {'mode_request_topic': 'robot_mode_requests'},
            {'path_request_topic': 'robot_path_requests'},
            {'destination_request_topic': 'robot_destination_requests'},
            {'dds_domain': 42},
            {'dds_robot_state_topic': 'robot_state'},
            {'dds_mode_request_topic': 'mode_request'},
            {'dds_path_request_topic': 'path_request'},
            {'dds_destination_request_topic': 'destination_request'},
            {'update_state_frequency': 20.0},
            {'publish_state_frequency': 2.0},
            ]
    )

    return LaunchDescription([fleet_server])