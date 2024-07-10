# Multi-Robot Simulation with Open-RMF
A multi-AGVs simulator with `open-rmf` on ROS2 `humble` with using `Gazebo` plugin named `slotcar`. This simulation is done by refering to packages `rmf_demos`.

Required packages:
- [`open-rmf`] (https://github.com/open-rmf/rmf)
- [`Gazebo Harmonic`] (https://gazebosim.org/docs)

## Gazebo simulation launching
To start the simulation and spawn robots, run the following command:

```bash
ros2 launch ign_sim simulation.launch.xml headless:=1
```
This command will launch gazebo with headless mode (no UI rendering)
You can change to `headless:=0' if you wish to see the UI

## Rviz for robot fleet visualization and fleet adapter
To start the `open_rmf` nodes and `fleet_adapter` with Rviz visualization window, run the following command:

```bash
ros2 launch ign_sim warehouse.launch.xml
```

## Open-RMF
Control Robot with `open_rmf`
- To send single destination to robots
For `go_to_palce`, it will move the robot to target position. `-F` = fleet name, `-R` = robot name, `-p` = destination
```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -F v1 -R AGV15 -p r60
```

For patrol, it will assign a robot with lowest cost to complete the task.`-p` = sequence of waypoint need, `-n` number of loop need to perform
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p r1 d1 r1 -n 1
```

You can also send random patrol task to the robot by:
```bash
ros2 launch ign_sim dispatch_patrol.py
```
This will create a random task to the robot fleet with 30 seconds interval

Data Logger
```bash
ros2 run ign_sim observer.py --task_state
```
Remarks: run this before launch the rviz and gazebo