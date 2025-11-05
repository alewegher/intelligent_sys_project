# int_sys_fp — Distributed Estimation with UWB on Multi-Robot (ROS 2 Humble)

Didactic ROS 2 package that simulates UWB distance sensing among three TurtleBot3 robots and fixed anchors, filters the measurements (Kalman Filter), and exposes data for a controller. It includes a complete Gazebo launch that spawns 3 robots and runs all nodes end-to-end.

## What this package does

This is a **didactic distributed estimation and control system** for multi-robot formations using UWB-based localization:

1. **Simulation setup**: Spawns 3 TurtleBot3 (burger) robots in Gazebo at configurable positions
2. **UWB emulation**: Emulates UWB distance sensors measuring:
   - Robot-to-anchor distances (3 fixed anchors)
   - Inter-robot distances (robot-to-robot)
   - Configurable noise models (Gaussian or Uniform)
3. **Distance filtering**: Kalman Filter node (Python) filters noisy UWB measurements and publishes clean distances
4. **Trilateration-based localization**: Controller node uses filtered distances to:
   - Compute triangle angles from distances (law of cosines)
   - Estimate robot positions via geometric trilateration using anchor positions
   - Calculate formation centroid from the 3 robot positions
5. **Formation control**: Dual PD controller architecture:
   - **Centroid tracker**: PD controller tracks a desired 2D trajectory (x,y) provided by a planner node
   - **Formation keeper**: PD controller maintains equilateral triangle configuration (equal inter-robot distances)
   - Final velocity commands combine both control terms
6. **Custom messages**: Ships RobotDist and AnchorDist message types for structured distance data
7. **Complete launch files**: Run the full stack (Gazebo + all nodes) or individual components

## Nodes and topics

- uwb_emulator (C++)
  - Publishes:
    - `/uwb/anchor_distances` — `int_sys_fp/msg/AnchorDist`
    - `/uwb/robot_distances` — `int_sys_fp/msg/RobotDist`
  - Parameters/Args:
    - `noise_type` (arg) 1=Gaussian (default), 2=Uniform
    - `uwb_sensor_frequency` (param) from launch arg `uwb_frequency`
    - `use_sim_time` (param) set true in simulation

- distance_kf_node (Python, executable `KF.py`)
  - Subscribes: `/uwb/anchor_distances`, `/uwb/robot_distances`
  - Publishes:
    - `/uwb/filtered_anchor_distances` — `int_sys_fp/msg/AnchorDist`
    - `/uwb/filtered_robot_distances` — `int_sys_fp/msg/RobotDist`
    - `/ukf_covariance_matrix` — `std_msgs/Float64MultiArray`
  - Reads config via ament index:
    - `share/int_sys_fp/sensor_params.yaml`

- controller_node (C++)
  - Subscribes: 
    - `/uwb/filtered_anchor_distances` — `int_sys_fp/msg/AnchorDist`
    - `/uwb/filtered_robot_distances` — `int_sys_fp/msg/RobotDist`
    - `/desired_trajectory` — `geometry_msgs/PoseStamped` (from planner, 2D trajectory)
  - Publishes:
    - `/cmd_vel` — velocity commands for robot 1
    - `/tb3_2/cmd_vel` — velocity commands for robot 2
    - `/tb3_3/cmd_vel` — velocity commands for robot 3
    - `/kf_filtered_robot_pose` — estimated robot positions
    - `/centroid_pose` — formation centroid position
  - Control architecture:
    1. Trilateration: computes robot positions from filtered UWB distances + anchor positions
    2. Centroid calculation: averages the 3 robot positions
    3. **PD Tracker**: tracks desired centroid trajectory (x,y) from planner
    4. **PD Formation**: maintains equilateral triangle (equal inter-robot distances)
    5. Combines both PD terms into final velocity commands for each robot
  - Uses gains from: `share/int_sys_fp/controller.yaml`
    - Position/velocity gains (Kp, Kv) per robot
    - Max velocity/angular velocity limits

## Launch files

- `synchronized_system.launch.py` — Full system
  - Starts Gazebo (server+client), spawns 3 robots, runs uwb_emulator, controller, and KF
  - Args:
    - `uwb_frequency` (default `50.0` Hz)
    - `noise_type` (default `1` = Gaussian)

- `uwb_emulator.launch.py` — Only the emulator (no Gazebo)
- `uwb_test_system.launch.py` — Emulator + PlotJuggler example

## Configuration files

- `sensor_params.yaml` — Anchor positions (3 fixed anchors), sensor frequency, Gaussian noise model (mean, stddev)
- `sensor_params_uniform.yaml` — Alternate uniform noise config (min, max range)
- `UKF_params.yaml` — Kalman Filter parameters (process noise, outlier detection)
- `controller.yaml` — Controller gains and limits:
  - `Kp_r1`, `Kp_r2`, `Kp_r3`: Proportional gains [x, y, z] per robot
  - `Kv_r1`, `Kv_r2`, `Kv_r3`: Velocity gains [x, y, z] per robot
  - `max_vel_r`: Maximum linear velocity (0.22 m/s for TurtleBot3)
  - `max_omega_r`: Maximum angular velocity (2.84 rad/s)

All configs are installed to `install/int_sys_fp/share/int_sys_fp/`.

## Requirements

- ROS 2 Humble on Ubuntu 22.04
- Gazebo (via `gazebo_ros`)
- TurtleBot3 Gazebo assets
- PlotJuggler (optional)

Suggested apt packages:

```zsh
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-turtlebot3-gazebo \
  ros-humble-plotjuggler \
  python3-lxml
```

## Build and install

From your workspace root (`~/ros2_ws`):

```zsh
# 1) Source ROS 2 (zsh)
source /opt/ros/humble/setup.zsh

# 2) Build only this package
colcon build --packages-select int_sys_fp

# 3) Source the workspace overlay (zsh)
source install/setup.zsh
```

If you ever see CMake cache mismatch errors, clean stale directories and rebuild:

```zsh
rm -rf build/int_sys_fp src/int_sys_fp/build install/int_sys_fp
colcon build --packages-select int_sys_fp
source install/setup.zsh
```

## Run the full system (Gazebo + 3 robots + UWB + KF)

```zsh
# Make sure you sourced ROS 2 and the workspace (see above)
ros2 launch int_sys_fp synchronized_system.launch.py \
  uwb_frequency:=50.0 \
  noise_type:=1   # 1=Gaussian, 2=Uniform
```

What you should see:
- Gazebo empty world opens
- Robots spawn at:
  - Robot 1: (0.0, 0.0)
  - Robot 2: (1.0, 0.0) namespace `tb3_2`
  - Robot 3: (0.0, 1.0) namespace `tb3_3`
- Nodes start: `uwb_emulator`, `controller_node`, `distance_kf_node`

## Monitor and debug

Useful commands:

```zsh
# Nodes
ros2 node list

# Topics
ros2 topic list | sort

# Peek at UWB raw distances
ros2 topic echo /uwb/anchor_distances --once
ros2 topic echo /uwb/robot_distances --once

# Peek at filtered distances
ros2 topic echo /uwb/filtered_anchor_distances --once
ros2 topic echo /uwb/filtered_robot_distances --once

# Covariance matrix (diagonal published as flattened matrix)
ros2 topic echo /ukf_covariance_matrix --once

# Check estimated robot positions and centroid
ros2 topic echo /kf_filtered_robot_pose --once
ros2 topic echo /centroid_pose --once

# Monitor robot velocity commands
ros2 topic echo /cmd_vel
ros2 topic echo /tb3_2/cmd_vel
ros2 topic echo /tb3_3/cmd_vel

# Send a test trajectory waypoint (planner node would do this)
ros2 topic pub --once /desired_trajectory geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}}}"

# Graph
ros2 run rqt_graph rqt_graph
```

## Troubleshooting

- Spawn service unavailable (Was Gazebo started with GazeboRosFactory?)
  - Wait a bit longer; the launch already delays spawns (6s, 10s, 13s). Increase if needed.
  - Ensure `gazebo_ros` is installed (`ros-humble-gazebo-ros`).

- TurtleBot3 world/assets missing
  - Install `ros-humble-turtlebot3-gazebo`.

- Package not found or executables missing
  - Rebuild and source overlay: `colcon build --packages-select int_sys_fp && source install/setup.zsh`

- KF.py not found or not executable
  - Ensure installed via CMake (it is). In source tree: `chmod +x src/int_sys_fp/src/KF.py` (one time), then rebuild.

- UWB emulator blocks waiting for input
  - It no longer reads from stdin; `noise_type` is passed as a launch arg and forwarded to the executable.

- CMake cache mismatch (paths mention old directories)
  - Clean: `rm -rf build/int_sys_fp src/int_sys_fp/build` and rebuild.

## Control architecture overview

The system implements a **distributed estimation + formation control** pipeline:

```
┌─────────────────┐
│ UWB Emulator    │ → Noisy distances (anchor + inter-robot)
└─────────────────┘
         ↓
┌─────────────────┐
│ Kalman Filter   │ → Filtered distances
└─────────────────┘
         ↓
┌─────────────────────────────────────────────────┐
│ Controller Node                                 │
│  1. Trilateration (distances → robot positions) │
│  2. Centroid calculation                        │
│  3. PD Tracker (centroid → desired trajectory)  │
│  4. PD Formation (maintain equilateral triangle)│
│  5. Combined velocity commands                  │
└─────────────────────────────────────────────────┘
         ↓
┌─────────────────┐
│ 3 TurtleBot3    │ → Execute velocity commands
└─────────────────┘
```

## Project structure (high level)

```
int_sys_fp/
  launch/
    synchronized_system.launch.py  # Full system (Gazebo + all nodes)
    uwb_emulator.launch.py         # UWB emulator only
    uwb_test_system.launch.py      # UWB + PlotJuggler
  src/
    UWB_utils_emulator.cpp          # uwb_emulator node (C++)
    Regulator_node.cpp              # controller_node (C++) - trilateration + dual PD control
    KF.py                           # distance_kf_node (Python) - Kalman filtering
  custom_messages/
    AnchorDist.msg                  # distances_a1, distances_a2, distances_a3
    RobotDist.msg                   # distances_r1, distances_r2, distances_r3
  include/
    UWB_utils_emulator.hpp          # C++ class declarations
    Regulator_node.hpp              # (if/when needed)
  *.yaml
    sensor_params.yaml              # Anchor positions, Gaussian noise
    sensor_params_uniform.yaml      # Uniform noise alternative
    UKF_params.yaml                 # KF process noise, outlier detection
    controller.yaml                 # PD gains (Kp, Kv), velocity limits
```

## License

Apache-2.0. See `src/int_sys_fp/LICENSE`.

## Acknowledgements

- TurtleBot3 Gazebo assets and examples
- ROS 2 Humble community packages
