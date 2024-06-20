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

    rviz_config_file = os.path.join(pkg_path, "config", "agv.rviz")

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
            #name="simple_path_planner",
            namespace=namespace,
            parameters=[{"robot_name":namespace},
                        {"initial_x":initial_position["x"]},
                        {"initial_y":initial_position["y"]}],
        )

        robot_state = Node(
            package="robot_control",
            executable="state_publisher",
            #name="simple_path_planner",
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

        battery_state = Node(
            package="robot_control",
            executable="battery_simulator",
            namespace=namespace,
        )

        fleet_client = Node(
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
            )
        
        all_robot_actions.extend([robot,robot_control,static_publisher,battery_state,robot_state,fleet_client])

    return LaunchDescription(all_robot_actions+[maps])