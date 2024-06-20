# Multi-Robot Simulation with simple navigator
A simple 2D multi-AGVs simulator with `Rviz2` on ROS2 `humble`, a simple navigator node called `simple_path_planner` was used to replace `nav2`. 
There are no any sensors plugin involved. The navigator moved based on changing the robot `base_footprint` at certain period of time with predefined speed. `battery_simulator` node had been created to calculate the battery level for both charging and draining. 

This packages also involved the application of `free_fleet` server and client, prepared for itegration with `open-rmf`

Required packages:
- [`free_fleet`] (https://github.com/open-rmf/free_fleet)
- [`open-rmf`] (https://github.com/open-rmf/rmf)
- [`nav2`] (https://github.com/ros-navigation/navigation2/tree/main) - this is for `nav2` `navigate_to_pose` interface required by `free_fleet`
- [`xacro`]

# Launching the Simulation

To start the simulation and spawn robots, each with its own navigation stack, run the following command:

```bash
ros2 launch robot_bringup robots.launch.py
```
This command initializes the robots in predefined namespaces and positions as configured in the robots.yaml file under robot_bringup/config.

### Navigating the Robots

To navigate the robots in RViz:

1. Open RViz.
2. In the Tools Properties panel, adjust the 2D Goal Pose topic to /{namespace}/goal_pose where {namespace} corresponds to the robot's namespace (AGV1, AGV2, AGV3, etc.).

This setup allows you to control each robot individually by setting goals within their respective namespaces.

3. Launch the navigation action client using CLI, for example for AGV1
```bash
ros2 run robot_control fake_action_client --ros-args -r __ns:=/AGV1
```
## Adding More Robots
To add more robots to the simulation:

1. Navigate to robot_bringup/config/robots.yaml.
2. Add a new entry for each robot with the following parameters:
   - namespace: Unique identifier for the robot (e.g., AGV8).
   - initial_pose_x: Starting x-coordinate of the robot.
   - initial_pose_y: Starting y-coordinate of the robot.
  
Example Entry:
```yaml
- namespace: AGV8
  initial_position:
    x: 3
    y: 0
```

# Visualizing Multiple Robots
All robots can be visualized simultaneously in a single RViz window, which facilitates monitoring and interaction across the simulated environment.


# Moving the Robot
For example: 
```bash
ros2 run ff_examples_ros2 send_destination_request.py -f v1 -r AGV1 -x 3 -y 2 --yaw 0.0 -i 11125 -l L1
```
This is the example to move robot `AGV1` in `v1` fleet at `L1` floor number to position `(3,2)` with facing angle `0`

# Open-RMF
To start simulation with open_rmf
- To start fleet manager, server and rviz
```bash
ros2 launch rmf_sim warehouse_sim.launch.xml
```
- To start the robots, in another terminal
```bash
ros2 launch robot_bringup robots.launch.py
```

![robots spawn and move gif] (https://github.com/s-w3i/warehouse_sim/blob/main/spawn.gif)

# TO-DO
1. Create own fleet adapter to remove the dependency of `free_fleet` to run the simulation. (Clients didnt register to server while client node bringup correctly, robot position not updated in fleet_state) 
![robots stop and disconnect gif] (https://github.com/s-w3i/warehouse_sim/blob/main/fleet_state.gif)
2. Expand the layout
3. Robot travelling time and distance record per task 