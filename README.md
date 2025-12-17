# int_sys_fp — Distributed Estimation with UWB on Multi-Robot (ROS 2 Humble)

Advanced ROS 2 package implementing **UWB-based localization** and **hierarchical formation control** for multi-robot systems. Features state-of-the-art filtering (Kalman with outlier rejection), analytical trilateration, and a two-phase control state machine optimized for differential-drive robots.

## What this package does

This is an **advanced distributed estimation and control system** for multi-robot formations using UWB-based localization:

1. **Simulation setup**: Spawns 3 TurtleBot3 Burger robots in Gazebo with realistic dynamics
2. **UWB distance sensing**: Emulates Ultra-Wideband (UWB) ranging sensors with:
   - Robot-to-anchor distances (3 fixed anchors for trilateration)
   - Inter-robot distances (peer-to-peer ranging for formation control)
   - Realistic noise models (Gaussian or Uniform) with configurable parameters
   - Configurable update rate (default 50 Hz)

3. **Robust distance filtering**: Dual-implementation Kalman Filter with outlier rejection
   - **Python KF** (`KF.py`): Original implementation with MAD-based outlier detection
   - **C++ KF** (`UKF.cpp`): High-performance version (default) with innovation gating
   - Filters non-Gaussian UWB noise and NLOS (Non-Line-of-Sight) errors
   - Publishes filtered distances + covariance estimates

4. **Analytical trilateration**: Direct closed-form position estimation
   - Solves circle intersection problem using Manolakis/Fang method
   - Converts 3 anchor distances → robot 2D position (x, y)
   - Handles singularities (collinear anchors) with determinant check
   - O(1) computational complexity for real-time operation

5. **Two-phase hierarchical control** with state machine:
   - **PHASE 1 - FORMATION**: Achieves equilateral triangle formation
     - Critically damped PD control (Kd = 2ζ√Kp, ζ=0.98)
     - No trajectory tracking (robots stay in place)
     - Convergence metric: ||[e₀₁, e₀₂, e₁₂]|| < 15cm
   - **PHASE 2 - TRACKING**: Trajectory following + formation maintenance
     - PID centroid tracking with anti-windup (back-calculation method)
     - Adaptive formation control (weighted by tracking error)
     - Smooth transition when formation is achieved

6. **Differential drive control**: Converts 2D velocity commands to (v, ω)
   - PD heading controller with derivative damping (Kp=2.5, Kd=0.8)
   - Adaptive speed reduction in turns (cosine-weighted, min 50%)
   - Saturation handling with velocity/angular limits
   - Odometry feedback for orientation (yaw extraction from quaternion)

7. **Formation control strategies**:
   - **Stiff mode** (FORMATION phase): Independent spring forces on each link
   - **Soft mode** (TRACKING phase): Weighted spring forces (30% when far from trajectory, 100% when close)
   - Symmetric distance averaging: (d_ij + d_ji)/2 for robustness

8. **Trajectory generation**: Circular path planner
   - Parametric circle with configurable radius and angular velocity
   - Velocity constraints respected (v_tangential = ω × r < v_max)
   - 50 Hz update rate synchronized with controller

9. **Custom ROS 2 messages**: Structured distance data types
   - `AnchorDist.msg`: 3 anchors × 3 robots = 9 distances
   - `RobotDist.msg`: 3 robots × 2 peers each = 6 distances

10. **Complete launch infrastructure**: Modular, parameter-driven launch files

## Nodes and topics

### uwb_emulator (C++)
Simulates UWB ranging sensors with realistic noise characteristics.

- **Publishes:**
  - `/uwb/anchor_distances` — `int_sys_fp/msg/AnchorDist` (raw noisy measurements)
  - `/uwb/robot_distances` — `int_sys_fp/msg/RobotDist` (raw noisy measurements)

- **Parameters/Args:**
  - `noise_type` (arg): 1=Gaussian (default), 2=Uniform
  - `uwb_sensor_frequency` (param): Update rate in Hz (default 50.0)
  - `use_sim_time` (param): true in simulation

- **Configuration:** `sensor_params.yaml` or `sensor_params_uniform.yaml`

---

### distance_kf_node (C++ or Python)
Filters UWB measurements using Kalman Filter with outlier rejection.

**C++ version (default, `distance_kf_node`):**
- **Algorithm:** Scalar Kalman Filter per distance measurement (15 total)
- **Outlier rejection:**
  1. Innovation gating: reject if |innovation| > 3√R (Mahalanobis distance)
  2. MAD filtering: Median Absolute Deviation on sliding window (10 samples)
- **Performance:** ~2x faster than Python, lower latency

**Python version (legacy, `KF.py`):**
- Same algorithm, prototyping-friendly

**Common interface:**
- **Subscribes:** `/uwb/anchor_distances`, `/uwb/robot_distances`
- **Publishes:**
  - `/uwb/filtered_anchor_distances` — `int_sys_fp/msg/AnchorDist`
  - `/uwb/filtered_robot_distances` — `int_sys_fp/msg/RobotDist`
- **Configuration:** `UKF_params.yaml` (process noise, outlier thresholds)

**To switch implementations:**
```bash
# Use C++ (default)
ros2 launch int_sys_fp synchronized_system.launch.py use_cpp_kf:=true

### `synchronized_system.launch.py` — Full system
Complete end-to-end simulation with all nodes.

**What it does:**
- Starts Gazebo (gzserver + gzclient)
- Spawns 3 TurtleBot3 Burger robots:
  - Robot 1: (0.0, 0.0, 0.01) → `/cmd_vel`, `/odom`
  - Robot 2: (1.0, 0.0, 0.01) → `/tb3_2/cmd_vel`, `/tb3_2/odom`
  - Robot 3: (0.0, 1.0, 0.01) → `/tb3_3/cmd_vel`, `/tb3_3/odom`
- Launches nodes: uwb_emulator, distance_kf_node (C++), controller_node, trajectory_planner

**Launch arguments:**
- `uwb_frequency` (default `50.0`): UWB sensor update rate in Hz
- `noise_type` (default `1`): 1=Gaussian, 2=Uniform noise
- `use_cpp_kf` (default `true`): Use C++ KF (faster) vs Python KF (legacy)
- `enable_planner` (default `true`): Enable circular trajectory planner

**Example usage:**
```bash
# Default (50 Hz UWB, Gaussian noise, C++ KF, with planner)
ros2 launch int_sys_fp synchronized_system.launch.py

# Custom: 100 Hz UWB, Uniform noise, Python KF, no planner
ros2 launch int_sys_fp synchronized_system.launch.py \
  uwb_frequency:=100.0 \
  noise_type:=2 \
  use_cpp_kf:=false \
  enable_planner:=false
```

---

### Other launch files

### controller_node (C++, `regulator_node`)
Implements hierarchical formation control with state machine.

- **Subscribes:**
  - `/uwb/filtered_anchor_distances` — `int_sys_fp/msg/AnchorDist`
  - `/uwb/filtered_robot_distances` — `int_sys_fp/msg/RobotDist`
  - `/desired_trajectory` — `geometry_msgs/Pose` (from planner)
  - `/odom`, `/tb3_2/odom`, `/tb3_3/odom` — `nav_msgs/Odometry` (orientation feedback)

- **Publishes:**
  - `/cmd_vel`, `/tb3_2/cmd_vel`, `/tb3_3/cmd_vel` — `geometry_msgs/Twist`
  - `/kf_filtered_robot_pose` — `std_msgs/Float64MultiArray` (6 values: [x₁,y₁, x₂,y₂, x₃,y₃])
  - `/centroid_position` — `std_msgs/Float64MultiArray` (2 values: [x_c, y_c])
  - `/tracking_error` — `std_msgs/Float64MultiArray` (2 values: [e_x, e_y])

- **Control Pipeline:**
  1. **Trilateration**: 3 anchor distances → robot position (x, y) using Fang's method
  2. **Centroid computation**: c = (p₁ + p₂ + p₃)/3
  3. **State machine** (automatic transition):
     - **FORMATION phase**: Stiff PD control to achieve equilateral triangle
       - Kp_formation = 2.0, Kd_formation = 2.77 (critically damped)
       - No trajectory tracking (desired_position ignored)
       - Metric: ||[e₀₁, e₀₂, e₁₂]|| where e_ij = (d_ij + d_ji)/2 - d_desired
       - Transition when error < 0.15m (15cm threshold)
     - **TRACKING phase**: PID centroid tracking + soft formation maintenance
       - PID gains: Kp=[1.5, 1.5], Kv=[0.8, 0.8], Ki=[0.3, 0.3]
       - Anti-windup: clamping + back-calculation (30% reduction on saturation)
       - Formation weight: 100% when close to trajectory, 30% when far
  4. **2D to differential drive conversion**:
     - Desired heading: θ_d = atan2(v_y, v_x)
     - Heading error: e_θ = θ_d - θ_current (from odometry)
     - Linear velocity: v = ||v_cmd|| × max(0.5, cos(e_θ)) (speed reduction in turns)
     - Angular velocity: ω = Kp_yaw × e_θ + Kd_yaw × ė_θ (PD control, Kp=2.5, Kd=0.8)
  5. **Saturation + anti-windup**: Clamps v, ω to TurtleBot3 limits, reduces integral term

- **Configuration:** `controller.yaml`
  - `control_frequency`: Control loop rate (default 50 Hz)
  - `position_gains` (Kp): Proportional gains for centroid tracking
  - `velocity_gains` (Kv): Derivative gains for centroid tracking
  - `integral_gains` (Ki): Integral gains for steady-state error elimination
  - `integral_limits`: Anti-windup saturation thresholds
  - `max_velocities`: TurtleBot3 physical limits (v_max=0.22 m/s, ω_max=2.84 rad/s)

---

### trajectory_planner (Python, `trajectory_planner.py`)
Generates circular reference trajectory for centroid tracking.

- **Publishes:**
  - `/desired_trajectory` — `geometry_msgs/Pose` (2D position, z=0)
All YAML configs are installed to `install/int_sys_fp/share/int_sys_fp/` and loaded at runtime.

### `sensor_params.yaml`
UWB sensor configuration with Gaussian noise model.

```yaml
UWB_sensor:
  frequency: 50.0  # Hz
  anchor 1:
    position: [0.0, 0.0, 0.0]  # Fixed anchor coordinates [x, y, z]
    noise model:
      mean: 0.0      # Gaussian mean (m)
      stddev: 0.05   # Standard deviation (m)
  # anchor 2, anchor 3, robot 1-3 similar structure
```

**Key parameters:**
- `position`: Anchor locations for trilateration (must not be collinear!)
- `stddev`: Measurement noise (σ = 5cm is realistic for UWB)

---

### `sensor_params_uniform.yaml`
Alternative configuration with uniform noise distribution.

```yaml
UWB_sensor:
  anchor 1:
    noise model:
      min: -0.1   # Uniform lower bound (m)
      max: 0.1    # Uniform upper bound (m)
```

**When to use:** Testing worst-case scenarios or non-Gaussian disturbances.

---

### `UKF_params.yaml`
Kalman Filter tuning parameters.

```yaml
process_noise:
  distance: 0.01     # Process noise for distance states (m²)
  rate: 0.1          # Process noise for rate-of-change (m²/s²)
outlier_detection:
  enabled: true
  threshold_sigma: 3.0  # Innovation gating: reject if > 3σ
  mad_window: 10        # Median Absolute Deviation window size
```

**Tuning guide:**
- `distance`: Lower → trust model more, higher → trust measurements more
- `threshold_sigma`: 3.0 = 99.7% confidence interval (standard choice)
- `mad_window`: Larger → more robust but slower adaptation

---

### `controller.yaml`
Hierarchical controller gains and constraints.

```yaml
controller_params:
  control_frequency: 50.0  # Control loop rate (Hz) - CRITICAL PARAMETER
  
  # PID gains for centroid tracking (TRACKING phase)
  position_gains:
    Kp_r1: [1.5, 1.5, 0.0]  # Proportional [x, y, z]
  velocity_gains:
    Kv_r1: [0.8, 0.8, 0.0]  # Derivative [x, y, z]
  integral_gains:
    Ki_r1: [0.3, 0.3, 0.0]  # Integral [x, y, z]
  
  # Anti-windup limits
  integral_limits:
    max_integral: [2.0, 2.0, 0.0]  # Maximum integral accumulation (m·s)
  
  # Physical constraints (TurtleBot3 Burger specs)
  max_velocities:
    max_vel_r: [0.22, 0.22, 0.0]    # Linear velocity limit (m/s)
    max_omega_r: [2.84, 2.84, 0.0]  # Angular velocity limit (rad/s)
```

**Tuning guide:**
- **Increase Kp**: Faster convergence, risk of oscillations
- **Increase Kv**: More damping, smoother motion
- **Increase Ki**: Eliminates steady-state error, risk of windup
- **control_frequency**: Must match trajectory planner rate (default 50 Hz)

**Formation control gains (hardcoded in controller_node.cpp):**
- FORMATION phase: Kp_formation = 2.0, Kd_formation = 2.77 (critically damped)
- TRACKING phase: Kf = 0.5 (soft spring constant)
- Heading control: Kp_yaw = 2.5, Kd_yaw = 0.822 m/s, reduce r or ω)
  - Safe config: r=2m, ω=0.1 → v=0.2 m/s ✓

- **Enable/disable in launch:**
  ```bash
  ros2 launch int_sys_fp synchronized_system.launch.py enable_planner:=false
  ```

### Basic inspection

```zsh
# List all nodes
ros2 node list

# List all topics (sorted)
ros2 topic list | sort

# Computational graph
ros2 run rqt_graph rqt_graph
```

### UWB measurements

```zsh
# Raw noisy distances (before filtering)
ros2 topic echo /uwb/anchor_distances --once
ros2 topic echo /uwb/robot_distances --once

# Filtered distances (after KF)
ros2 topic echo /uwb/filtered_anchor_distances --once
ros2 topic echo /uwb/filtered_robot_distances --once

# Monitor update rate
ros2 topic hz /uwb/anchor_distances  # Should be ~50 Hz
```

### State estimation

```zsh
# Robot positions from trilateration (6 values: x₁,y₁, x₂,y₂, x₃,y₃)
ros2 topic echo /kf_filtered_robot_pose

# Formation centroid (2 values: x_c, y_c)
ros2 topic echo /centroid_position

# Tracking error (2 values: e_x, e_y)
ros2 topic echo /tracking_error
### Build/Installation Issues

**Problem:** Package not found or executables missing
```zsh
# Solution: Clean rebuild
rm -rf build/int_sys_fp install/int_sys_fp log/latest_build
colcon build --packages-select int_sys_fp
source install/setup.zsh
```

**Problem:** CMake cache mismatch (paths mention old directories)
```zsh
# Solution: Full clean
rm -rf build/ install/ log/
colcon build --packages-select int_sys_fp
```

**Problem:** `KF.py` or `trajectory_planner.py` not executable
```zsh
# Solution: Fix permissions (one-time)
chmod +x src/int_sys_fp/src/KF.py
chmod +x src/int_sys_fp/src/trajectory_planner.py
colcon build --packages-select int_sys_fp
```

---

### Gazebo/Simulation Issues

**Problem:** Spawn service unavailable (timeout waiting for `/spawn_entity`)
- **Cause:** Gazebo not fully initialized
- **Solution:** Wait longer (launch uses 6s/10s/13s delays) or increase `TimerAction` periods in launch file

**Problem:** TurtleBot3 models not found
```zsh
# Solution: Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
export TURTLEBOT3_MODEL=burger  # Also set in launch file
```

**Problem:** Gazebo black screen or crash
- **Cause:** Graphics driver issues or insufficient resources
- **Solution:** Run gzserver only (headless):
  ```zsh
  # Edit launch file: comment out gzclient_launch
  ```


```
int_sys_fp/
├── launch/
│   ├── synchronized_system.launch.py   # Main: Gazebo + all nodes + planner
│   ├── uwb_emulator.launch.py          # UWB emulator standalone
│   └── uwb_test_system.launch.py       # UWB + PlotJuggler for debugging
│
├── src/
│   ├── UWB_utils_emulator.cpp          # UWB sensor emulation (C++)
│   ├── UKF.cpp                          # Kalman Filter with outlier rejection (C++)
│   ├── KF.py                            # Kalman Filter (Python, legacy)
│   ├── Regulator_node.cpp               # Hierarchical controller (C++)
│   │                                    #   - Trilateration
│   │                                    #   - State machine (FORMATION/TRACKING)
│   │                                    #   - PID centroid tracking
│   │                                    #   - Formation control
│   │                                    #   - Differential drive conversion
│   └── trajectory_planner.py            # Circular trajectory generator (Python)
│
├── include/
│   └── UWB_utils_emulator.hpp          # Header for UWB emulator
│
├── custom_messages/
│   ├── AnchorDist.msg                   # [distances_a1, distances_a2, distances_a3]
│   │                                    #  Each: array of 3 floats (robot distances)
│   └── RobotDist.msg                    # [distances_r1, distances_r2, distances_r3]
│                                        #  Each: array of 2 floats (peer distances)
│
├── CMakeLists.txt                       # Build configuration
├── package.xml                          # ROS 2 package manifest
│
└── Configuration files (*.yaml):
    ├── sensor_params.yaml               # Anchor positions, Gaussian noise (σ=5cm)
    ├── sensor_params_uniform.yaml       # Alternative: uniform noise (±10cm)
    ├── UKF_params.yaml                  # KF tuning: process noise, outlier thresholds
    └── controller.yaml                  # Control gains and limits
                                         #   - PID: Kp, Kv, Ki
                                         #   - Frequency: 50 Hz
                                         #   - Limits: v_max, ω_max
                                         #   - Anti-windup: max_integral
```

### Key Components

**Control Node (`Regulator_node.cpp`):**
- **Lines 23-120**: Initialization, parameter loading, state machine setup
- **Lines 252-282**: `trilaterate_robot_position()` — Fang's analytical method
- **Lines 294-320**: `compute_formation_error()` — Symmetric distance averaging
- **Lines 323-350**: `compute_centroid_tracking_control()` — PID with anti-windup
- **Lines 353-408**: `compute_formation_control()` — Dual-mode (stiff/soft)
- **Lines 411-562**: `control_loop()` — Main state machine + actuation

**Kalman Filter (`UKF.cpp`):**
- **Lines 83-148**: Initialization: 15 scalar filters (9 anchor + 6 robot distances)
- **Lines 195-227**: `update()` — Main filter loop
- **Lines 290-327**: `filter_single_distance()` — Scalar KF with innovation gating
- **Lines 330-357**: `apply_advanced_filtering()` — MAD-based outlier rejection

**UWB Emulator (`UWB_utils_emulator.cpp`):**
- Simulates ranging sensors with configurable noise
- Publishes raw measurements at 50 Hz

**Trajectory Planner (`trajectory_planner.py`):**
- Parametric circle: `x = r·cos(ωt)`, `y = r·sin(ωt)`
- Velocity constraint enforcement*Cause:** Trajectory velocity exceeds robot capabilities
- **Solution:** Reduce planner velocity: v_tangential = ω × r
  ```yaml
  # In launch file, planner parameters:
  radius: 2.0            # Reduce from 5.0
  angular_velocity: 0.08  # Reduce from 0.1
  # → v = 0.08 × 2.0 = 0.16 m/s < 0.22 m/s ✓
  ```

**Problem:** Robots drift apart during tracking (formation breaks)
- **Cause:** Formation weight too low or Kf too small
- **Solution:** Increase formation gain in controller code
  ```cpp
  // In Regulator_node.cpp, line ~94
  Kf = 0.8;  // Increase from 0.5
  ```

---

### Sensor/Estimation Issues

**Problem:** Position estimates jump or diverge
- **Cause:** Outliers not filtered or anchor geometry poor
- **Solution:**
  1. Check anchor positions are not collinear (determinant check will warn)
  2. Enable/verify outlier detection in `UKF_params.yaml`:
     ```yaml
     outlier_detection:
       enabled: true
       threshold_sigma: 3.0
     ```

**Problem:** KF covariances explode (> 10.0)
- **Cause:** Process noise too high or no measurements
- **Solution:** Reduce process noise in `UKF_params.yaml`:
  ```yaml
  process_noise:
    distance: 0.005  # Reduce from 0.01
  ```

**Problem:** High-frequency jitter in filtered distances
- **Cause:** MAD window too small or measurement rate too high
- **Solution:** Increase MAD window in KF code:
  ```cpp
  // In UKF.cpp, line ~100
  static constexpr int MAX_HISTORY_LENGTH = 20;  // Increase from 10
  ```
References

### State Estimation
1. **Manolakis, D. E.** (1996). *Efficient solution and performance analysis of 3-D position estimation by trilateration.* IEEE Transactions on Aerospace and Electronic Systems, 32(4), 1239-1248.
   - Analytical trilateration used in `trilaterate_robot_position()`

2. **Fang, B. T.** (1986). *Trilateration and extension to Global Positioning System navigation.* Journal of Guidance, Control, and Dynamics, 9(6), 715-717.
   - Closed-form solution for circle intersection

3. **Thomas, F., & Ros, L.** (2005). *Revisiting trilateration for robot localization.* IEEE Transactions on Robotics, 21(1), 93-101.
   - Geometric singularities and error analysis

### Filtering & Outlier Rejection
4. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T.** (2001). *Estimation with Applications to Tracking and Navigation.* John Wiley & Sons.
   - Innovation gating (Mahalanobis distance) used in KF

5. **Leys, C., et al.** (2013). *Detecting outliers: Do not use standard deviation around the mean, use absolute deviation around the median.* Journal of Experimental Social Psychology, 49(4), 764-766.
   - MAD (Median Absolute Deviation) outlier detection

6. **Ting, J. A., Theodorou, E., & Schaal, S.** (2007). *A Kalman filter for robust outlier detection.* IEEE/RSJ IROS, pp. 1514-1519.
   - Robust Kalman filtering strategies

### Formation Control
7. **Ren, W., & Beard, R. W.** (2008). *Distributed Consensus in Multi-vehicle Cooperative Control.* Springer.
   - Formation control theory and consensus algorithms

8. **Lewis, M. A., & Tan, K. H.** (1997). *High precision formation control of mobile robots using virtual structures.* Autonomous Robots, 4(4), 387-403.
   - Virtual structure approach (centroid tracking)

### Control Theory
9. **Åström, K. J., & Murray, R. M.** (2008). *Feedback Systems: An Introduction for Scientists and Engineers.* Princeton University Press.
   - PID control, anti-windup strategies

10. **Franklin, G. F., Powell, J. D., & Emami-Naeini, A.** (2019). *Feedback Control of Dynamic Systems* (8th ed.). Pearson.
    - Critically damped systems (ζ=0.98), pole placement

## Acknowledgements

- TurtleBot3 Gazebo assets and examples (ROBOTIS)
- ROS 2 Humble community packages
- Gazebo simulation framework
- Eigen3 linear algebra library

**Problem:** Control loop running slow (< 50 Hz)
- **Cause:** CPU overload or C++ KF not being used
- **Solution:**
  1. Use C++ KF: `use_cpp_kf:=true` (default)
  2. Reduce UWB frequency: `uwb_frequency:=25.0`
  3. Check CPU usage: `top -p $(pgrep regulator_node)`

**Problem:** Gazebo physics running slow (< real-time)
- **Cause:** Too many robots or complex world
- **Solution:**
  1. Reduce `max_step_size` in Gazebo world file
  2. Use fewer sensors (already using minimal TurtleBot3)
  3. Run headless (no gzclient)

---

### Common Warnings/Errors

**Warning:** `Large innovation in measurement X: Y m`
- **Meaning:** Outlier detected, measurement rejected
- **Action:** Normal for UWB (NLOS effects), no action needed unless frequent

**Error:** `Trilateration singular matrix for robot X`
- **Meaning:** Anchors are collinear or nearly collinear
- **Action:** Reposition anchors in `sensor_params.yaml` to form non-degenerate triangle

**Warning:** `Saturation detected - reducing integral error`
- **Meaning:** Anti-windup active (velocity limits reached)
- **Action:** Normal during aggressive maneuvers, reduce desired velocity if persistent

**Info:** `PHASE TRANSITION: FORMATION → TRACKING`
- **Meaning:** Formation achieved, now tracking trajectory
- **Action:** Success! Monitor `/tracking_error` to verify tracking performance

```zsh
# Watch controller logs for phase transitions
ros2 node list | grep controller  # Get node name
ros2 topic echo /rosout | grep "PHASE TRANSITION"

# Expected output when formation achieved:
# "=== PHASE TRANSITION: FORMATION → TRACKING ==="
# "Formation achieved! Error: 0.142 m < 0.150 m"
```

### Manual trajectory commands

```zsh
# Send a single waypoint (for testing without planner)
ros2 topic pub --once /desired_trajectory geometry_msgs/Pose \
  "{position: {x: 2.0, y: 2.0, z: 0.0}}"

# Move centroid to origin
ros2 topic pub --once /desired_trajectory geometry_msgs/Pose \
  "{position: {x: 0.0, y: 0.0, z: 0.0}}"

# Send continuous circular motion (0.1 Hz, 5m radius)
# (Better to use trajectory_planner node instead)
```

### Performance metrics

```zsh
# Node CPU usage (requires htop or top)
top -p $(pgrep -f regulator_node)

# Message latency (end-to-end delay)
ros2 topic echo /uwb/anchor_distances --field header.stamp
ros2 topic echo /cmd_vel --field header.stamp  # Compare timestamps

# Formation error (check inter-robot distances)
ros2 topic echo /uwb/filtered_robot_distances
# distances_r1[0] should ≈ distances_r2[0] ≈ desired_distance (1.5m)
```

### Visualization with PlotJuggler

```zsh
# Install if not already
sudo apt install ros-humble-plotjuggler-ros

# Launch and subscribe to topics
ros2 run plotjuggler plotjuggler

# Recommended plots:
# - /centroid_position vs /desired_trajectory (XY scatter plot)
# - /tracking_error (time series of ||e||)
# - /cmd_vel/linear/x and /cmd_vel/angular/z (control effort)
# - /uwb/filtered_anchor_distances/distances_a1 (distance measurements)
```

### Advanced debugging

```zsh
# Check TF tree (if using tf2)
ros2 run tf2_tools view_frames

# Record a bag for offline analysis
ros2 bag record -a -o test_run

# Replay bag at 0.5x speed
ros2 bag play test_run.db3 -r 0.5

# Inspect node parameters
ros2 param list /controller_node
ros2 param get /controller_node use_sim_time
┌──────────────────────────────────────────────────────────────────┐
│ SENSING LAYER                                                    │
├──────────────────────────────────────────────────────────────────┤
│ UWB Emulator (C++)                                               │
│  • Simulates ranging sensors (3 anchors × 3 robots = 15 meas.)   │
│  • Adds realistic noise (Gaussian σ=5cm or Uniform ±10cm)        │
│  • Publishes at 50 Hz                                            │
└──────────────────────────────────────────────────────────────────┘
         ↓ /uwb/anchor_distances, /uwb/robot_distances
┌──────────────────────────────────────────────────────────────────┐
│ FILTERING LAYER                                                  │
├──────────────────────────────────────────────────────────────────┤
│ Kalman Filter with Outlier Rejection (C++ or Python)            │
│  • Scalar KF per distance (15 independent filters)               │
│  • Innovation gating: |innovation| < 3√R                         │
│  • MAD filtering: sliding window (10 samples)                    │
│  • Publishes filtered distances + covariances                    │
└──────────────────────────────────────────────────────────────────┘
         ↓ /uwb/filtered_anchor_distances, /uwb/filtered_robot_distances
┌──────────────────────────────────────────────────────────────────┐
│ ESTIMATION LAYER                                                 │
├──────────────────────────────────────────────────────────────────┤
│ Analytical Trilateration                                         │
│  • Fang's method: 3 distances → 2D position                      │
│  • Solves: Ax + By = C, Dx + Ey = F (closed-form)               │
│  • Outputs: robot positions p₁, p₂, p₃                           │
└──────────────────────────────────────────────────────────────────┘
         ↓ p₁, p₂, p₃ (internal state)
┌──────────────────────────────────────────────────────────────────┐
│ CONTROL LAYER (State Machine)                                   │
├──────────────────────────────────────────────────────────────────┤
│ ┌──────────────────────────────────────────────────────────────┐ │
│ │ PHASE 1: FORMATION (Initial)                                 │ │
│ │ ┌────────────────────────────────────────────────────────┐   │ │
│ │ │ • Compute formation error: ||[e₀₁, e₀₂, e₁₂]||        │   │ │
│ │ │ • Stiff PD control: Kp=2.0, Kd=2.77 (crit. damped)    │   │ │
│ │ │ • No trajectory tracking (centroid stays in place)     │   │ │
│ │ │ • Each robot: F_i = Kp·e₁·d₁ + Kp·e₂·d₂               │   │ │
│ │ └────────────────────────────────────────────────────────┘   │ │
│ │  Transition: error < 0.15m (15cm threshold)                  │ │
│ │              ↓                                                │ │
│ │ ┌────────────────────────────────────────────────────────┐   │ │
│ │ │ PHASE 2: TRACKING (Steady-state)                       │   │ │
│ │ │ • Centroid: c = (p₁ + p₂ + p₃)/3                       │   │ │
│ │ │ • PID tracking: u_c = Kp·e + Ki·∫e + Kv·ė            │   │ │
│ │ │   - Kp=[1.5,1.5], Kv=[0.8,0.8], Ki=[0.3,0.3]         │   │ │
│ │ │   - Anti-windup: clamp + back-calc (30% on sat.)      │   │ │
│ │ │ • Formation: F_i = Kf·e₁·d₁ + Kf·e₂·d₂ (soft)        │   │ │
│ │ │   - Weight: 100% near traj, 30% far from traj         │   │ │
│ │ │ • Total: u_i = u_c + w·F_i                            │   │ │
│ │ └────────────────────────────────────────────────────────┘   │ │
│ └──────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
         ↓ u = [u_x, u_y] (2D velocity command per robot)
┌──────────────────────────────────────────────────────────────────┐
│ ACTUATION LAYER (Differential Drive Conversion)                 │
├──────────────────────────────────────────────────────────────────┤
│ For each robot i:                                                │
│  1. Desired heading: θ_d = atan2(u_y, u_x)                       │
│  2. Heading error: e_θ = θ_d - θ_current (from odometry)         │
│  3. Linear velocity: v = ||u|| × max(0.5, cos(e_θ))             │
│     → Speed reduction: 50-100% based on alignment                │
│  4. Angular velocity: ω = Kp_yaw·e_θ + Kd_yaw·ė_θ               │
│     → PD control: Kp=2.5, Kd=0.8 (damped rotation)               │
│  5. Saturation: clamp v ≤ 0.22 m/s, |ω| ≤ 2.84 rad/s            │
└──────────────────────────────────────────────────────────────────┘
         ↓ cmd_vel: [v, 0, ω] per robot
┌──────────────────────────────────────────────────────────────────┐
│ ROBOT DYNAMICS (Gazebo Simulation)                              │
│  • TurtleBot3 Burger: differential drive                         │
│  • Wheel radius: 0.033m, wheelbase: 0.16m                        │
│  • Realistic friction, inertia, noise                            │
└──────────────────────────────────────────────────────────────────┘
```

### Key Design Choices

1. **Two-phase control**: Separate formation acquisition from trajectory tracking
   - Avoids coupling between formation error and tracking error
   - Ensures stable initial conditions before high-speed maneuvers

2. **Critically damped formation control**: ζ=0.98 (slightly underdamped)
   - Fast convergence without oscillations
   - Kd = 2ζ√Kp ensures optimal damping ratio

3. **PID with anti-windup**: Back-calculation method
   - Integral term eliminates steady-state error (centroid offset)
   - Saturation detection → reduce integral by 70% to prevent windup

4. **PD heading control**: Derivative damping for smooth rotations
   - Kd=0.8 prevents oscillatory yaw behavior common in P-only control
   - Finite difference approximation: ė_θ ≈ (e_θ - e_θ_prev)/Δt

5. **Adaptive formation weighting**: Context-aware control allocation
   - When far from trajectory (||e_track|| > 0.2m): prioritize tracking (30% formation)
   - When close to trajectory: maintain formation (100% weight)

6. **Symmetric distance averaging**: Robust to measurement asymmetry
   - d_avg = (d_ij + d_ji)/2 reduces bias from directional errors

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
