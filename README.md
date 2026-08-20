# int_sys_fp — Distributed Pose Estimation and Formation Control for Multi-Robot Systems (ROS 2 Humble)

ROS 2 package implementing **UWB+IMU pose estimation** (EKF/UKF) and **distributed hierarchical formation control** for a 3-robot TurtleBot3 swarm. Each robot runs its own estimator and controller instance; robots exchange only pose estimates and FSM state — there is no central node computing commands for all three robots.

## What this package does

1. **Simulation setup**: Spawns 3 TurtleBot3 Burger robots in Gazebo (standard `turtlebot3_gazebo` model, including its IMU sensor).
2. **UWB distance sensing**: Emulates UWB ranging (`uwb_emulator`) — robot-to-anchor distances (3 fixed anchors) and inter-robot distances, with configurable Gaussian or Uniform noise at 50 Hz.
3. **Per-robot pose estimation** (`pose_ekf_node` / `pose_ukf_node`, one instance per robot, `robot_id` 0/1/2):
   - State `[x, y, θ]`, unicycle process model, predicted from the robot's own `cmd_vel` (linear velocity) and IMU gyro (angular velocity).
   - Correction from UWB distances (3 anchors + 2 neighbors, using the neighbors' last received pose estimate) **and** IMU orientation for θ — UWB distances alone do not observe θ, so IMU fusion is required.
   - Standard one-step Riccati recursion (`P_pred = F·P·F' + Q`, gain recomputed every cycle) — not a steady-state DARE solve, since `F_k`/`H_k` are time-varying.
   - Init: zero-mean state, inflated initial covariance (`P0 = p0_scale·I`) that self-compensates as measurements arrive.
   - Publishes both a standard `PoseWithCovarianceStamped` (for the controller) and a full-transparency `PoseEstimateDebug` message (state, full covariance, raw/predicted measurements, innovation, noise matrices — meant for bag recording and offline analysis, e.g. in MATLAB).
4. **Distributed formation + trajectory control** (`regulator_node`, one instance per robot, `robot_id` 0/1/2):
   - Local two-phase state machine: **FORMATION** (stiff critically-damped PD to reach an equilateral triangle) → **TRACKING** (PID centroid tracking + soft formation maintenance).
   - Transition to TRACKING requires both: local formation error below threshold for a debounce window, **and** an AND-consensus with the two neighbors (via `fsm_state` broadcast) — so all three robots switch together rather than at three uncoordinated times.
   - Transition is gain-blended over a configurable ramp window (not an instantaneous switch) to reduce the tracking-error peaks that occur at mode switching.
5. **Legacy distance-space Kalman Filter** (`distance_kf_node`, from `KF.cpp`/`KF.py`): the original 15-scalar-filter implementation (filters raw UWB distances, no pose/dynamics model). Kept as an optional baseline for comparison, not used by the controllers. Launch with `enable_legacy_kf:=true`.
6. **Trajectory generation**: circular reference path (`trajectory_planner.py`), broadcast to all robots as a shared centroid target.

## Architecture overview

```
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│ TB3 robot 0  │   │ TB3 robot 1  │   │ TB3 robot 2  │   (Gazebo, standard turtlebot3_burger model)
│ /cmd_vel     │   │/tb3_2/cmd_vel│   │/tb3_3/cmd_vel│
│ /odom /imu   │   │/tb3_2/odom.. │   │/tb3_3/odom.. │
└──────┬───────┘   └──────┬───────┘   └──────┬───────┘
       │                  │                  │
┌──────▼───────┐   ┌──────▼───────┐   ┌──────▼───────┐
│pose_ekf/ukf_ │   │pose_ekf/ukf_ │   │pose_ekf/ukf_ │   UWB (anchor+robot distances, broadcast)
│  node_0      │◄─►│  node_1      │◄─►│  node_2      │   + own cmd_vel + own IMU
│ [x,y,θ] EKF  │   │ [x,y,θ] EKF  │   │ [x,y,θ] EKF  │   → publishes pose_estimate + pose_debug
└──────┬───────┘   └──────┬───────┘   └──────┬───────┘
       │ pose_estimate (own + 2 neighbors)    │
┌──────▼───────┐   ┌──────▼───────┐   ┌──────▼───────┐
│controller_   │   │controller_   │   │controller_   │   fsm_state (AND-consensus between the 3)
│  node_0      │◄─►│  node_1      │◄─►│  node_2      │   /desired_trajectory (shared, from planner)
│ local FSM    │   │ local FSM    │   │ local FSM    │
└──────┬───────┘   └──────┬───────┘   └──────┬───────┘
       ▼                  ▼                  ▼
    cmd_vel            cmd_vel            cmd_vel
```

There is no node that loops over all 3 robots: `regulator_node` and `pose_ekf_node`/`pose_ukf_node` are each launched 3 times, once per `robot_id`, and only exchange pose/FSM broadcasts with each other.

## Nodes and topics

### `uwb_emulator` (C++)
- **Publishes:** `/uwb/anchor_distances`, `/uwb/robot_distances` — `int_sys_fp/msg/AnchorDist`, `int_sys_fp/msg/RobotDist` (raw, noisy).
- **Config:** `sensor_params.yaml` (or `sensor_params_uniform.yaml`), launch arg `noise_type` (1=Gaussian, 2=Uniform), param `uwb_sensor_frequency`.

### `pose_ekf_node` / `pose_ukf_node` (C++, one instance per robot via `robot_id` param)
- **Subscribes:** `{prefix}/cmd_vel`, `{prefix}/imu`, `/uwb/anchor_distances`, `/uwb/robot_distances` (own row selected via `robot_id`), and the two neighbors' `{neighbor_prefix}/pose_estimate`.
- **Publishes:**
  - `{prefix}/pose_estimate` — `geometry_msgs/msg/PoseWithCovarianceStamped`: x, y, θ (as quaternion z/w), partial 6×6 covariance (only the x/y/θ block is filled). Consumed by `regulator_node`.
  - `{prefix}/pose_debug` — `int_sys_fp/msg/PoseEstimateDebug`: `robot_id`, `x`, `y`, `theta` (plain doubles), `p[9]` (full 3×3 covariance, row-major), `z[6]`/`z_pred[6]`/`innovation[6]` (layout: anchor0, anchor1, anchor2, neighbor0, neighbor1, theta_imu), `q_diag[3]`, `r_diag[6]`, `have_imu`, `v_cmd`, `omega_imu`, `dt`. This is the topic to record in a bag for offline/MATLAB analysis — every field is named, no index-guessing.
- **Config:** `pose_filter_params.yaml` (Q, initial covariance scale, UKF sigma-point params, synthetic IMU orientation noise std, gain mode), `sensor_params.yaml` (anchor positions + R for anchors/neighbors).
- **Note on θ and the IMU:** the Gazebo `gazebo_ros_imu_sensor` plugin never populates `orientation_covariance` (unimplemented in the plugin, verified from its source — the field is always zero regardless of SDF config), so synthetic noise is added in software (`imu_orientation_noise_std`) before using orientation as a correction; `angular_velocity_covariance` *is* populated from the real configured sensor noise and is used directly as `R_ω`.

### `regulator_node` (C++, one instance per robot via `robot_id` param)
- **Subscribes:** `{prefix}/pose_estimate` (own + 2 neighbors), `{neighbor_prefix}/fsm_state` (2 neighbors), `/desired_trajectory` (shared, broadcast).
- **Publishes:**
  - `{prefix}/cmd_vel` — `geometry_msgs/msg/Twist`.
  - `{prefix}/fsm_state` — `int_sys_fp/msg/FsmState`: `phase` (0=FORMATION, 1=TRACKING), `formation_error`.
  - `{prefix}/tracking_error`, `{prefix}/desired_trajectory_array`, `{prefix}/centroid_position` — `std_msgs/msg/Float64MultiArray` (debug/PlotJuggler).
- **Control pipeline:** formation error from estimated positions → local debounce + neighbor AND-consensus → gain-blended ramp (formation-stiff-PD ↔ tracking-PID+soft-formation) over `phase_transition_ramp_s` → 2D velocity → differential-drive conversion (heading PD, speed reduction in turns, saturation + anti-windup).
- **Config:** `controller.yaml`.

### `distance_kf_node` (C++ `KF.cpp` / Python `KF.py`, legacy baseline, single instance)
- Filters the 15 raw UWB distances directly (scalar KF per distance, no pose/dynamics model, MAD-based outlier rejection). Not consumed by `regulator_node` or the pose filters — launched only with `enable_legacy_kf:=true`, for report/comparison purposes.
- **Subscribes:** `/uwb/anchor_distances`, `/uwb/robot_distances`. **Publishes:** `/uwb/filtered_anchor_distances`, `/uwb/filtered_robot_distances`.
- **Config:** `UKF_params.yaml`.

### `trajectory_planner` (Python)
- **Publishes:** `/desired_trajectory` — `geometry_msgs/msg/Pose` (circular reference path, shared centroid target).

## Configuration files

All installed to `install/int_sys_fp/share/int_sys_fp/` and loaded at runtime (no rebuild needed to change values).

| File | Used by | Contents |
|---|---|---|
| `sensor_params.yaml` | `uwb_emulator`, `pose_ekf_node`, `pose_ukf_node`, legacy `distance_kf_node` | Anchor positions, per-anchor/per-robot UWB noise (Gaussian, stddev) |
| `sensor_params_uniform.yaml` | (alternative to above) | Same structure, Uniform noise |
| `pose_filter_params.yaml` | `pose_ekf_node`, `pose_ukf_node` | Process noise `Q` (x/y/theta variance), `P0` inflation scale, UKF sigma-point params (α, β, κ), synthetic IMU orientation noise std, `gain_mode` |
| `UKF_params.yaml` | legacy `distance_kf_node` only | Process noise, outlier detection (MAD threshold, window) for the 15-scalar legacy filter |
| `controller.yaml` | `regulator_node` | Control frequency, PID gains (Kp/Kv/Ki), integral limits, velocity limits, `phase_transition_ramp_s` (FSM gain-blending window) |

## Launch

```bash
ros2 launch int_sys_fp synchronized_system.launch.py \
  filter_type:=ekf \          # 'ekf' or 'ukf' — pose filter used by all 3 robots
  uwb_frequency:=50.0 \
  noise_type:=1 \              # 1=Gaussian, 2=Uniform
  enable_planner:=true \
  enable_legacy_kf:=false      # also launch the legacy distance_kf_node baseline
```

This spawns Gazebo + 3 TurtleBot3 robots (robot 0 at origin unnamespaced, robots 1/2 under `/tb3_2`, `/tb3_3`), the UWB emulator, 3 `regulator_node` instances, 3 `pose_ekf_node` or `pose_ukf_node` instances (per `filter_type`), and the trajectory planner.

## Monitoring and debugging

```bash
ros2 node list
ros2 topic list | sort

# Raw UWB
ros2 topic echo /uwb/anchor_distances --once
ros2 topic echo /uwb/robot_distances --once

# Pose estimate (controller input) and full debug channel (record this one for MATLAB)
ros2 topic echo /pose_estimate --once
ros2 topic echo /pose_debug --once
ros2 topic echo /tb3_2/pose_debug --once   # robot 1
ros2 topic echo /tb3_3/pose_debug --once   # robot 2

# FSM state and consensus
ros2 topic echo /fsm_state
ros2 topic echo /tb3_2/fsm_state

# Watch phase transitions in the logs
ros2 topic echo /rosout | grep "PHASE TRANSITION"

# Record a bag for offline/MATLAB analysis
ros2 bag record /pose_debug /tb3_2/pose_debug /tb3_3/pose_debug \
  /fsm_state /tb3_2/fsm_state /tb3_3/fsm_state \
  /tracking_error /tb3_2/tracking_error /tb3_3/tracking_error \
  -o test_run

# Graph
ros2 run rqt_graph rqt_graph
```

## Troubleshooting

- **Spawn service unavailable** (timeout waiting for `/spawn_entity`): wait longer, Gazebo needs to fully initialize before the 6s/10s/13s spawn delays fire.
- **TurtleBot3 models not found**: `sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo`.
- **Package not found / executables missing**: `colcon build --packages-select int_sys_fp && source install/setup.bash`.
- **CMake cache mismatch**: `rm -rf build/int_sys_fp install/int_sys_fp log/latest_build && colcon build --packages-select int_sys_fp`.
- **A `regulator_node`/`pose_ekf_node`/`pose_ukf_node` instance exits immediately with "robot_id parameter not set"**: these nodes require an explicit `robot_id` (0/1/2) parameter — this is intentional (forces distributed configuration), the provided launch file already sets it for each of the 3 instances.
- **Robots drift apart during tracking**: increase `Kf` (formation gain during TRACKING) in `Regulator_node.cpp`, or shorten `phase_transition_ramp_s` in `controller.yaml`.
- **θ estimate drifts or stays near 0**: check `/imu` is actually publishing (`ros2 topic echo /imu --once`) — pose filters need it for θ correction, UWB distances alone cannot observe θ.
- **Pose covariance never shrinks / diverges**: check `pose_filter_params.yaml`'s `Q` isn't too large relative to the measurement noise in `sensor_params.yaml`.

## Project structure

```
int_sys_fp/
├── launch/
│   └── synchronized_system.launch.py   # Gazebo + 3× (regulator_node, pose_ekf/ukf_node) + uwb_emulator + planner
├── src/
│   ├── UWB_utils_emulator.cpp          # uwb_emulator
│   ├── PoseEKF_node.cpp                # pose_ekf_node — per-robot pose EKF
│   ├── UKF.cpp                         # pose_ukf_node — per-robot pose UKF
│   ├── Regulator_node.cpp              # regulator_node — per-robot distributed controller + local FSM
│   ├── KF.cpp / KF.py                  # distance_kf_node — legacy 15-scalar distance filter (baseline only)
│   └── trajectory_planner.py           # shared circular reference trajectory
├── include/
│   ├── pose_dynamics.hpp               # shared unicycle f()/F() + UWB+IMU measurement h()/H(), used by both filters
│   └── UWB_utils_emulator.hpp
├── custom_messages/
│   ├── AnchorDist.msg, RobotDist.msg   # UWB distance vectors
│   ├── FsmState.msg                    # phase + formation_error (FSM consensus broadcast)
│   └── PoseEstimateDebug.msg           # full filter transparency channel (state, P, z, z_pred, innovation, Q, R)
├── CMakeLists.txt, package.xml
└── *.yaml
    ├── sensor_params.yaml / sensor_params_uniform.yaml
    ├── pose_filter_params.yaml         # Q, P0, UKF params, IMU noise (editable without rebuild)
    ├── UKF_params.yaml                 # legacy filter only
    └── controller.yaml                 # PID gains, limits, phase_transition_ramp_s
```

## References

### State Estimation
1. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T.** (2001). *Estimation with Applications to Tracking and Navigation.* John Wiley & Sons.
2. **Wan, E. A., & Van Der Merwe, R.** (2000). *The unscented Kalman filter for nonlinear estimation.* IEEE AS-SPCC.

### Filtering & Outlier Rejection
3. **Leys, C., et al.** (2013). *Detecting outliers: Do not use standard deviation around the mean, use absolute deviation around the median.* Journal of Experimental Social Psychology, 49(4), 764-766.
4. **Ting, J. A., Theodorou, E., & Schaal, S.** (2007). *A Kalman filter for robust outlier detection.* IEEE/RSJ IROS.

### Trilateration (legacy pipeline)
5. **Manolakis, D. E.** (1996). *Efficient solution and performance analysis of 3-D position estimation by trilateration.* IEEE TAES, 32(4), 1239-1248.
6. **Fang, B. T.** (1986). *Trilateration and extension to Global Positioning System navigation.* JGCD, 9(6), 715-717.
7. **Thomas, F., & Ros, L.** (2005). *Revisiting trilateration for robot localization.* IEEE T-RO, 21(1), 93-101.

### Formation Control
8. **Ren, W., & Beard, R. W.** (2008). *Distributed Consensus in Multi-vehicle Cooperative Control.* Springer.
9. **Lewis, M. A., & Tan, K. H.** (1997). *High precision formation control of mobile robots using virtual structures.* Autonomous Robots, 4(4), 387-403.

### Control Theory
10. **Åström, K. J., & Murray, R. M.** (2008). *Feedback Systems: An Introduction for Scientists and Engineers.* Princeton University Press.
11. **Franklin, G. F., Powell, J. D., & Emami-Naeini, A.** (2019). *Feedback Control of Dynamic Systems* (8th ed.). Pearson.

## License

Apache-2.0. See `src/int_sys_fp/LICENSE`.

## Acknowledgements

- TurtleBot3 Gazebo assets and examples (ROBOTIS)
- ROS 2 Humble community packages
- Gazebo simulation framework
- Eigen3 linear algebra library
