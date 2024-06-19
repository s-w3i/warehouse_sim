import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    namespaces = ['AGV1', 'AGV2', 'AGV3', 'AGV4']
    
    delay = 5.0  # Delay in seconds between each client launch
    clients = []

    for i, namespace in enumerate(namespaces):
        clients.append(TimerAction(
            period=i * delay,
            actions=[Node(
                package='free_fleet_client_ros2',
                executable='free_fleet_client_ros2',
                name=f"{namespace}_ff_client_node",
                namespace=namespace,
                output='both',
                parameters=[
                    {'fleet_name': 'v1'},
                    {'robot_name': namespace},
                    {'robot_model': 'cloudy'},
                    {'level_name': 'L1'},
                    {'dds_domain': 42},
                    {'max_dist_to_first_waypoint': 10.00},
                    {'map_frame': 'map'},
                    {'robot_frame': f'{namespace}/base_footprint'},
                    {'nav2_server_name': f'/{namespace}/navigate_to_pose'},
                    {'battery_state_topic': f'/{namespace}/battery_state'},
                    {'update_frequency': 20.0},
                    {'publish_frequency': 2.0},
                    {'use_sim_time': False}
                ]
            )]
        ))

    return LaunchDescription(clients)