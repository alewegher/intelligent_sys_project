# int_sys_fp — Distributed Estimation with UWB on Multi-Robot (ROS 2 Humble)

Didactic ROS 2 package that simulates UWB distance sensing among three TurtleBot3 robots and fixed anchors, filters the measurements (Kalman Filter), and exposes data for a controller. It includes a complete Gazebo launch that spawns 3 robots and runs all nodes end-to-end.

## What this package does

- Spawns 3 TurtleBot3 (burger) robots in Gazebo at configurable positions
- Emulates UWB distance sensors to anchors and inter-robot distances (C++)
- Filters distances with a dedicated Kalman Filter node (Python)
- Provides a controller node scaffold (C++) that subscribes to UWB topics
- Ships custom message types for distances (RobotDist, AnchorDist)
- Offers launch files to run the full stack or just the emulator

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
  - Subscribes: `/uwb/anchor_distances`, `/uwb/robot_distances`
  - Planned to use gains from: `share/int_sys_fp/controller.yaml`
  - Currently contains scaffolding for future control logic

## Launch files

- `synchronized_system.launch.py` — Full system
  - Starts Gazebo (server+client), spawns 3 robots, runs uwb_emulator, controller, and KF
  - Args:
    - `uwb_frequency` (default `50.0` Hz)
    - `noise_type` (default `1` = Gaussian)

- `uwb_emulator.launch.py` — Only the emulator (no Gazebo)
- `uwb_test_system.launch.py` — Emulator + PlotJuggler example

## Configuration files

- `sensor_params.yaml` — Anchor positions, frequency, Gaussian noise defaults
- `sensor_params_uniform.yaml` — Alternate uniform noise config
- `UKF_params.yaml` — UKF/KF parameters (currently KF uses defaults and sensor params)
- `controller.yaml` — Controller gains and limits (parsed by controller node in future work)

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

## Project structure (high level)

```
int_sys_fp/
  launch/
    synchronized_system.launch.py
    uwb_emulator.launch.py
    uwb_test_system.launch.py
  src/
    UWB_utils_emulator.cpp     # uwb_emulator node (C++)
    Regulator_node.cpp         # controller_node (C++)
    KF.py                      # distance_kf_node (Python)
  custom_messages/
    AnchorDist.msg
    RobotDist.msg
  include/
    UWB_utils_emulator.hpp     # (C++ class declarations)
    Regulator_node.hpp         # (if/when needed)
  *.yaml
    sensor_params.yaml
    sensor_params_uniform.yaml
    UKF_params.yaml
    controller.yaml
```

## License

Apache-2.0. See `src/int_sys_fp/LICENSE`.

## Acknowledgements

- TurtleBot3 Gazebo assets and examples
- ROS 2 Humble community packages
