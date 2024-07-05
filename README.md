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
