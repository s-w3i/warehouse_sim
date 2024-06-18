import os
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap, Node

def generate_launch_description():
    try:
        pkg_path = get_package_share_directory("robot_bringup")
    except PackageNotFoundError as e:
        print(f"Package not found: {e}")
        return LaunchDescription([])  # Return an empty launch description on error

    yaml_file = os.path.join(pkg_path, "config", "robots.yaml")
    try:
        with open(yaml_file, "r") as f:
            robots_config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Configuration file not found: {yaml_file}")
        return LaunchDescription([])
    except yaml.YAMLError as e:
        print(f"Error reading YAML file: {e}")
        return LaunchDescription([])

    all_robot_actions = []
    for config in robots_config["robots"]:
        namespace = config["namespace"]
        initial_position = config["initial_position"]

        maps = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_path, "launch"), "/map.launch.py"]),
            #launch_arguments={"namespace": namespace}.items(),
        )
        
        robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_path, "launch"), "/robot_bringup.launch.py"]),
            launch_arguments={"namespace": namespace, "frame_prefix": f"{namespace}/"}.items(),
        )

        robot_control = Node(
            package="robot_control",
            executable="fake_action_server",
            name="simple_path_planner",
            namespace=namespace,
            parameters=[{"robot_name":namespace},
                        {"initial_x":initial_position["x"]},
                        {"initial_y":initial_position["y"]}],
        )

        static_publisher = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=[str(initial_position["x"]), str(initial_position["y"]), "0", "0", "0", "0", f"map", f"{namespace}/base_footprint"],
            namespace=namespace,
            parameters=[{"use_sim_time": False}],
            #remappings=remappings
        )

        free_fleet_client = Node(
        package='free_fleet_client_ros2',
        executable='free_fleet_client_ros2',
        name= "ff_client_node",
        namespace = namespace,
        output='both',
        parameters=[{'fleet_name': 'v1'},
                    {'robot_name': namespace},
                    {'robot_model': 'cloudy'},
                    {'level_name': 'L1'},
                    {'dds_domain': 42},
                    {'max_dist_to_first_waypoint': 10.00},
                    {'map_frame': 'map'},
                    {'robot_frame': f'{namespace}/base_footprint'},
                    {'nav2_server_name': f'/{namespace}/navigate_to_pose'},
                    #{'battery_state_topic': battery_state},
                    #{'update_frequency': 5.0},
                    #{'publish_frequency':5.0},
                    {'use_sim_time': False}])
        
        all_robot_actions.extend([robot,robot_control,static_publisher,free_fleet_client])

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


    return LaunchDescription(all_robot_actions+[maps,fleet_server])