# CityLab — Autonomous Robot Patrol

A ROS 2 project written in C++ that uses LiDAR data to navigate around obstacles. The `robot_patrol` package can be run in a simulated environment with RViz or against a physical robot.

## Patrolling
[partolling.mov](robot_patrol/assets/partolling.mp4)

## RViz and the simulated environment
[rviz in simulation.mov](robot_patrol/assets/rviz%20in%20simulation.mp4)

## Overview

The patrol node subscribes to laser scans, checks the surrounding space for obstacles, and publishes velocity commands to keep the robot moving while avoiding blocked paths.

**Tech stack:** ROS 2 · C++ (`rclcpp`) · `sensor_msgs/LaserScan` · `geometry_msgs/Twist` · RViz2

## Project structure

```text
citylab_project/
└── robot_patrol/
    ├── src/
    │   └── patrol.cpp
    ├── launch/
    │   └── start_patrolling.launch.py
    ├── rviz/
    │   └── patrol.rviz
    ├── CMakeLists.txt
    └── package.xml
```

## Prerequisites

- A working ROS 2 environment with `colcon` and the package dependencies installed.
- The CityLab project inside the `src` folder of a ROS 2 workspace.
- For simulation: the simulator running and RViz2 available.
- For the real robot: an active connection to the robot, with laser scan and velocity-command ROS topics available.

## Build

From your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_patrol
source install/setup.bash
```

> Run `source install/setup.bash` in each new terminal before launching the package. Rebuild after changing the C++ code or launch configuration.

## Run in simulation (with RViz)

**1.** Start the robot simulation in your simulation environment.

**2.** Build and source the workspace using the commands above.

**3.** Launch the patrol node and RViz together:

```bash
ros2 launch robot_patrol start_patrolling.launch.py
```

The launch file starts `patrol_node` and opens RViz2 with the project's `rviz/patrol.rviz` configuration.

The RViz configuration is set up to display:

| Display | ROS topic / frame |
| --- | --- |
| Laser scan | `/scan` |
| Odometry | `/odom` |
| Robot model | `/fastbot_1_robot_description` |
| Fixed frame | `fastbot_1_odom` |

If the laser scan does not appear in RViz, check the topic and try the **Best Effort** reliability setting for the LaserScan display.

## Run on the real robot

**1.** Connect to the real robot and start its required robot-side systems/drivers. Make sure the robot is in a safe, clear area and that its emergency stop is accessible.

**2.** If you're using the `real-robot` Git branch for the physical robot, switch to it **before** building:

```bash
cd ~/ros2_ws/src/citylab_project
git switch real-robot
cd ~/ros2_ws
colcon build --packages-select robot_patrol
source install/setup.bash
```

If you already have the correct code checked out, just use the commands in **Build**.

**3.** Check that the robot is publishing its laser data:

```bash
ros2 topic list -t
ros2 topic info /scan
```

**4.** Run the patrol node without RViz:

```bash
ros2 run robot_patrol patrol_node
```

The robot should respond to laser readings and publish movement commands while the node is running. Check `ros2 topic info /cmd_vel` in another terminal once the node starts. **Ctrl+C** stops the node, but do not assume it stops the motors: verify the robot is stationary and use its emergency stop when necessary.

## ROS topics and environment configuration

The topic names are selected in `robot_patrol/src/patrol.cpp` using `is_construct_environment_`.

| Environment | Laser scan subscription | Velocity command publication |
| --- | --- | --- |
| The Construct / standard robot setup | `/scan` | `/cmd_vel` |
| Local Gazebo `vehicle_green` setup | `/vehicle_green/scan` | `/model/vehicle_green/cmd_vel` |

> **Important:** `is_construct_environment_` distinguishes the topic configurations; it is **not** a universal simulation-versus-physical-robot switch. Check the actual ROS topics in the environment you're running and select the matching configuration before building.

To inspect topic types and data:

```bash
ros2 topic list -t
ros2 topic echo /scan --once
ros2 topic info /cmd_vel
```

## Troubleshooting

| Problem | What to check |
| --- | --- |
| `Package 'robot_patrol' not found` | Build the package and run `source ~/ros2_ws/install/setup.bash`. |
| RViz opens without the saved layout | Check that `rviz/patrol.rviz` is installed and that the launch file passes its correct path using `-d`. |
| No laser scan appears in RViz | Confirm the scan topic, the RViz fixed frame, TF availability, and LaserScan QoS (try **Best Effort**). |
| Robot does not move | Check that the correct `/cmd_vel` topic is being published, the robot accepts commands, and valid laser scans are arriving. |
| Robot turns the wrong way or hits obstacles | Inspect scan angles, sector mapping, and the clearance readings printed by `patrol_node`. |

Stop a running launch or node with **Ctrl+C**.

---

**Package:** `robot_patrol` · **Node:** `patrol_node` · **Launch file:** `start_patrolling.launch.py`
