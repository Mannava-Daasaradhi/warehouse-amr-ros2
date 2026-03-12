# Configuration Files Explained

This document explains every parameter in the three YAML configuration files used by the Warehouse AMR.

---

## Table of Contents

1. [ekf.yaml — Extended Kalman Filter](#1-ekfyaml--extended-kalman-filter)
2. [ros2_controllers.yaml — Differential Drive Controller](#2-ros2_controllersyaml--differential-drive-controller)
3. [nav2_params.yaml — Navigation Stack](#3-nav2_paramsyaml--navigation-stack)
4. [warehouse_map.yaml — Map Metadata](#4-warehouse_mapyaml--map-metadata)

---

## 1. `ekf.yaml` — Extended Kalman Filter

> **Package:** `robot_localization`  
> **Node:** `ekf_filter_node`  
> **Purpose:** Fuse wheel odometry and IMU data into a single smooth, drift-reduced pose estimate.

### Why Do We Need an EKF?

Wheel odometry alone accumulates error over time (wheels slip, encoders have noise).
The IMU provides high-frequency orientation data but drifts on linear axes.
The EKF **fuses** both sensors, taking the best parts of each:
- Wheel odometry → good for short-term linear velocity
- IMU → good for angular velocity (yaw rate)

The result (`/odometry/filtered`) is what Nav2 uses for all its planning.

### General Parameters

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `use_sim_time` | `true` | Use Gazebo's `/clock` topic as the time source (required in simulation) |
| `frequency` | `30.0` | How often (Hz) the EKF runs its prediction-correction cycle |
| `sensor_timeout` | `0.1` | If a sensor hasn't published for this many seconds, use the last known value |
| `two_d_mode` | `true` | Constrain the robot to the XY plane; ignore Z position, roll, and pitch |
| `publish_tf` | `true` | Whether the EKF publishes the `odom → base_footprint` TF transform |

### Frame IDs

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `map_frame` | `map` | The global map frame name (used for context; EKF does not publish map→odom) |
| `odom_frame` | `odom` | The odometry frame (parent of `base_footprint` in the TF tree) |
| `base_link_frame` | `base_footprint` | The robot's reference frame (the EKF computes `odom → base_footprint`) |
| `world_frame` | `odom` | Tells the EKF which frame is the fixed reference; `odom` = no map correction |

### Odometry Input (`odom0`)

```yaml
odom0: /diff_drive_controller/odom
```

This is the raw wheel-encoder odometry published by the differential drive controller.

The `odom0_config` is a 15-element boolean array controlling which of the
15 state variables are read from this odometry source:

```
[x,    y,    z,       # position
 roll, pitch, yaw,    # orientation
 vx,   vy,   vz,     # linear velocity
 vroll,vpitch,vyaw,  # angular velocity
 ax,   ay,   az]     # linear acceleration
```

| Index | State | Enabled? | Reason |
|-------|-------|----------|--------|
| 6 (vx) | Linear velocity X | ✔ | Wheel odometry is reliable for forward/backward speed |
| 7 (vy) | Linear velocity Y | ✔ | Used even though a diff-drive can't strafe (set by convention) |
| 11 (vyaw) | Angular velocity Z | ✔ | Yaw rate from encoder difference is useful |
| All others | — | ✗ | Either not measured or better provided by IMU |

### IMU Input (`imu0`)

```yaml
imu0: /imu/data_raw
```

The raw IMU data from the Gazebo IMU plugin (100 Hz).

`imu0_config`:

| Index | State | Enabled? | Reason |
|-------|-------|----------|--------|
| 5 (yaw) | Orientation Z | ✔ | IMU gives an absolute yaw estimate (from integrated gyro) |
| 11 (vyaw) | Angular velocity Z | ✔ | Gyroscope yaw rate |
| All others | — | ✗ | IMU accelerometers and gyros for X/Y are not needed for 2-D motion |

```yaml
imu0_remove_gravitational_acceleration: true
```
Removes the 9.81 m/s² gravitational component from the accelerometer reading.
Without this, the Z-axis acceleration would be permanently offset.

---

## 2. `ros2_controllers.yaml` — Differential Drive Controller

> **Package:** `ros2_controllers` (loaded via `gz_ros2_control`)  
> **Nodes:** `controller_manager`, `joint_state_broadcaster`, `diff_drive_controller`  
> **Purpose:** Drive the two powered wheels and publish joint encoder data.

### Controller Manager

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100   # control loop runs at 100 Hz
```

The `controller_manager` is the central hub that loads and updates all controllers
at a fixed rate.  100 Hz is fast enough for smooth motion without excessive CPU load.

### Registered Controllers

```yaml
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster
diff_drive_controller:
  type: diff_drive_controller/DiffDriveController
```

These two controllers are registered with the manager but not yet activated here.
They are activated by the `spawner` nodes in `sim.launch.py` (with a 15-second delay
to give Gazebo time to initialise the hardware interface).

### Diff Drive Controller Parameters

#### Wheel Configuration

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `left_wheel_names` | `["left_wheel_joint"]` | Name of the left wheel joint in the URDF |
| `right_wheel_names` | `["right_wheel_joint"]` | Name of the right wheel joint in the URDF |
| `wheel_separation` | `0.52` m | Distance between wheel centres; **must match URDF `wheel_sep`** |
| `wheel_radius` | `0.10` m | Wheel radius; **must match URDF `wheel_radius`** |

> **Critical:** `wheel_separation` and `wheel_radius` are used to compute odometry
> from encoder counts.  If they differ from the URDF, the odometry will be wrong and
> the robot will navigate to incorrect positions.

#### Multipliers (calibration)

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `wheel_separation_multiplier` | `1.0` | Scale factor to compensate for mechanical inaccuracies |
| `left_wheel_radius_multiplier` | `1.0` | Scale factor for left wheel wear/inconsistency |
| `right_wheel_radius_multiplier` | `1.0` | Scale factor for right wheel wear/inconsistency |

These are all 1.0 in simulation; on a real robot you would tune them after measuring
actual vs predicted travel distance.

#### Odometry Publishing

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `publish_rate` | `50.0` Hz | How often `/diff_drive_controller/odom` is published |
| `odom_frame_id` | `odom` | Parent frame of the odometry message |
| `base_frame_id` | `base_footprint` | Child frame (the robot) |
| `open_loop` | `false` | Use encoder feedback (closed-loop) rather than integrating commands |
| `enable_odom_tf` | `true` | Publish the `odom → base_footprint` TF transform |

> **Note:** When `enable_odom_tf: true`, both the diff-drive controller **and** the
> EKF node would publish `odom → base_footprint`.  The EKF overrides the controller's
> TF because it is more accurate.  To avoid duplicate transforms, you can set
> `enable_odom_tf: false` once the EKF is configured.

#### Safety

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `cmd_vel_timeout` | `0.5` s | If no `/cmd_vel` command arrives for 0.5 s, stop the robot |
| `publish_limited_velocity` | `true` | Publish the actual limited velocity command so downstream nodes can read it |
| `use_stamped_vel` | `false` | Accept `geometry_msgs/Twist` (not TwistStamped) — but `twist_stamper` converts for us |

#### Velocity and Acceleration Limits

| Axis | Limit Type | Value | Explanation |
|------|-----------|-------|-------------|
| Linear X | max velocity | ±0.8 m/s | Maximum forward/backward speed |
| Linear X | max acceleration | ±2.5 m/s² | Maximum speed-up / braking rate |
| Angular Z | max velocity | ±2.0 rad/s | Maximum turning speed (~115°/s) |
| Angular Z | max acceleration | ±3.2 rad/s² | Maximum angular acceleration |

These limits are enforced by the controller **and** replicated in `velocity_smoother`
and `nav2_params.yaml` so the whole stack agrees on physical limits.

---

## 3. `nav2_params.yaml` — Navigation Stack

> **Packages:** `nav2_*`  
> **Purpose:** Configure every Nav2 server: planner, controller, costmaps, localisers, behaviours.

### 3.1 AMCL (Adaptive Monte Carlo Localisation)

Used when `slam:=false`.  AMCL estimates where the robot is within a pre-built map
using a particle filter.  Particles represent possible robot poses; they are weighted
by how well the LiDAR scan matches the map at each candidate pose.

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `alpha1–alpha4` | `0.2` | Noise model for odometry translation and rotation errors |
| `alpha5` | `0.1` | Noise model for strafe (unused for diff-drive but required) |
| `base_frame_id` | `base_footprint` | Robot frame AMCL tracks |
| `global_frame_id` | `map` | The map frame |
| `min_particles` | `500` | Minimum number of particles (more = more accurate but slower) |
| `max_particles` | `3000` | Maximum number of particles |
| `laser_model_type` | `likelihood_field` | Uses a pre-computed distance field; faster than beam model |
| `set_initial_pose` | `true` | Seed particles at a known starting location instead of spreading globally |
| `initial_pose.x/y` | `0.5 / 0.5` | Starting position (matches robot spawn in `sim.launch.py`) |
| `initial_pose.yaw` | `−1.5707963` | −90° = facing −Y direction (matches robot spawn yaw) |
| `update_min_d` | `0.25` m | Only update particles when robot moves this far (saves CPU) |
| `update_min_a` | `0.2` rad | Only update when robot rotates this much (~11°) |
| `transform_tolerance` | `1.0` s | How much TF timestamp lag is allowed before rejecting a transform |

### 3.2 BT Navigator (Behaviour Tree Navigator)

The `bt_navigator` is the top-level action server that `mission_executor.py` calls
via `BasicNavigator.goToPose()`.  It orchestrates the full navigation pipeline using
a Behaviour Tree XML file.

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `global_frame` | `map` | Planning happens in the map frame |
| `robot_base_frame` | `base_link` | The robot's body frame |
| `odom_topic` | `/odometry/filtered` | Subscribes to EKF output for the robot's current pose |
| `bt_loop_duration` | `10` ms | How often the Behaviour Tree ticks (100 Hz) |
| `default_server_timeout` | `20` s | How long to wait for action servers to become available |
| `default_nav_to_pose_bt_xml` | (ROS path) | Which Behaviour Tree XML to use; this one includes recovery behaviours (spin, backup) |

### 3.3 Controller Server (Local Planner)

The controller server runs the **Regulated Pure Pursuit** local planner that converts
the global path into wheel velocity commands.

#### Top-Level

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `controller_frequency` | `20.0` Hz | How often the controller computes new velocity commands |
| `failure_tolerance` | `0.3` s | How long to tolerate controller failure before triggering recovery |

#### Goal Checker (`general_goal_checker`)

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `xy_goal_tolerance` | `0.15` m | Consider goal reached if within 15 cm of target position |
| `yaw_goal_tolerance` | `0.1` rad | Consider goal reached if within ~5.7° of target heading |
| `stateful` | `true` | Once within tolerance, stay "succeeded" even if the robot drifts slightly |

#### Regulated Pure Pursuit Controller

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `desired_linear_vel` | `0.8` m/s | Cruise speed along straight segments |
| `lookahead_dist` | `0.6` m | The point on the path the robot steers toward |
| `min_lookahead_dist` | `0.3` m | Minimum lookahead (prevents oscillation at low speed) |
| `max_lookahead_dist` | `0.9` m | Maximum lookahead (prevents cutting corners too aggressively) |
| `rotate_to_heading_angular_vel` | `1.2` rad/s | Speed to rotate in-place before starting to drive |
| `use_rotate_to_heading` | `true` | Rotate to face the path direction before moving |
| `allow_reversing` | `false` | The robot only drives forward |
| `max_angular_accel` | `3.2` rad/s² | Matches the ros2_controllers angular acceleration limit |
| `use_collision_detection` | `true` | Stop if the path ahead has an obstacle (before `collision_monitor`) |

### 3.4 Local Costmap

The local costmap is a rolling 5×5 m window centred on the robot.
Nav2 uses it for real-time obstacle avoidance.

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `global_frame` | `odom` | Local costmap is expressed in the `odom` frame (moves with the robot) |
| `rolling_window` | `true` | Window follows the robot |
| `width / height` | `5` m | Size of the local costmap window |
| `resolution` | `0.05` m | Each cell represents a 5 cm × 5 cm area |
| `footprint` | `[[0.30,0.25],...]` | Polygon describing the robot's footprint (30 cm front/back, 25 cm left/right from centre) |
| `footprint_padding` | `0.03` m | Extra 3 cm safety margin added around the footprint |
| `voxel_layer` | enabled | Marks obstacles seen by LiDAR |
| `inflation_layer` | enabled | Expands obstacles by `inflation_radius = 0.45` m (keeps robot away from walls) |

### 3.5 Global Costmap

The global costmap covers the entire map and is used by the path planner.

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `global_frame` | `map` | Global costmap is fixed to the map |
| `resolution` | `0.05` m | Same resolution as the SLAM map |
| `static_layer` | enabled | Loads walls and obstacles from the SLAM/map_server map |
| `obstacle_layer` | enabled | Adds sensor-detected obstacles on top of the static map |
| `inflation_layer` | `inflation_radius = 0.55` m | Slightly larger inflation than local costmap for conservative global paths |

### 3.6 Planner Server (Global Planner)

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `expected_planner_frequency` | `20.0` Hz | Target planning rate |
| `plugin` | `NavfnPlanner` | Uses Dijkstra's / A* to find the shortest collision-free path |
| `tolerance` | `0.5` m | If no path is found within 0.5 m of the goal, accept a slightly further point |
| `use_astar` | `false` | Use Dijkstra (false) instead of A* (true); Dijkstra guarantees optimal paths |
| `allow_unknown` | `true` | Allow planning through unmapped (unknown) cells |

### 3.7 Smoother Server

Post-processes the global path to remove sharp corners.

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `simple_smoother` | `nav2_smoother::SimpleSmoother` | Iterative smoothing algorithm |
| `tolerance` | `1e-10` | Convergence threshold (very tight for maximum smoothing) |
| `max_its` | `1000` | Maximum smoothing iterations per path |
| `do_refinement` | `true` | Do a second smoothing pass for even smoother paths |

### 3.8 Behaviour Server

Recovery behaviours executed when navigation gets stuck.

| Behaviour | Plugin | What It Does |
|-----------|--------|-------------|
| `spin` | `nav2_behaviors::Spin` | Rotates 360° in-place to sense the environment |
| `backup` | `nav2_behaviors::BackUp` | Drives backward a short distance |
| `drive_on_heading` | `nav2_behaviors::DriveOnHeading` | Drives forward on current heading |
| `wait` | `nav2_behaviors::Wait` | Pauses for a configurable duration |
| `assisted_teleop` | `nav2_behaviors::AssistedTeleop` | Allows human-controlled movement while preventing collisions |

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `max_rotational_vel` | `1.0` rad/s | Maximum speed during spin recovery |
| `rotational_acc_lim` | `3.2` rad/s² | Acceleration limit during recovery (matches controller limit) |
| `simulate_ahead_time` | `2.0` s | How far ahead to simulate the robot path when checking safety |

### 3.9 Velocity Smoother

Low-pass filters velocity commands to respect acceleration limits.
This sits between the controller and `collision_monitor`.

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `smoothing_frequency` | `20.0` Hz | Output rate |
| `max_velocity` | `[0.8, 0.0, 2.0]` | Max [linear_x, linear_y, angular_z] — matches controller limits |
| `min_velocity` | `[-0.8, 0.0, -2.0]` | Min velocities (negative = reverse) |
| `max_accel` | `[2.5, 0.0, 3.2]` | Acceleration limits (m/s² and rad/s²) |
| `max_decel` | `[-2.5, 0.0, -3.2]` | Deceleration limits |
| `odom_topic` | `odometry/filtered` | EKF output used to measure actual velocity |
| `feedback` | `OPEN_LOOP` | Compute smoothed command from previous command, not measured velocity |

### 3.10 Collision Monitor

The final safety gate before velocity commands reach the wheels.
If an obstacle enters the monitored polygon, the robot slows or stops.

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `cmd_vel_in_topic` | `cmd_vel_smoothed` | Input from velocity smoother |
| `cmd_vel_out_topic` | `cmd_vel` | Output to `twist_stamper` → wheels |
| `FootprintApproach` | type=polygon, action=approach | Slows the robot proportionally when obstacles approach the footprint |
| `time_before_collision` | `2.0` s | Start slowing down when collision is predicted within 2 seconds |
| scan source: `min_height` | `0.15` m | Ignore scan returns below 15 cm (floor reflections) |
| scan source: `max_height` | `2.0` m | Ignore scan returns above 2 m (ceiling lights) |

### 3.11 Waypoint Follower

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `stop_on_failure` | `false` | Continue to the next waypoint even if the current one fails |
| `waypoint_pause_duration` | `200` ms | Wait time at each waypoint |

> **Note:** `mission_executor.py` does **not** use `waypoint_follower`.  It uses
> `BasicNavigator.goToPose()` directly (one goal at a time).  `waypoint_follower`
> is available for use cases that need to send the entire sequence at once.

### 3.12 SLAM Toolbox

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| `mode` | `mapping` | Build a new map (vs. `localization` mode which loads an existing map) |
| `resolution` | `0.05` m | Map cell size (matches costmap resolution) |
| `max_laser_range` | `12.0` m | Maximum LiDAR range used for mapping |
| `map_update_interval` | `5.0` s | Rebuild the `/map` message every 5 seconds |
| `transform_publish_period` | `0.02` s | Publish `map → odom` TF at 50 Hz |
| `minimum_travel_distance` | `0.5` m | Don't process a new scan unless the robot has moved 0.5 m |
| `minimum_travel_heading` | `0.5` rad | Or unless it has rotated ~28° |
| `do_loop_closing` | `true` | When the robot revisits a known area, correct accumulated drift |
| `solver_plugin` | `CeresSolver` | Uses Google Ceres for scan-matching optimisation |
| `stack_size_to_use` | `40000000` | 40 MB stack for the Ceres solver thread |

---

## 4. `warehouse_map.yaml` — Map Metadata

> **File:** `src/warehouse_amr/maps/warehouse_map.yaml`  
> **Used by:** `map_server` (when `slam:=false`)

```yaml
image: warehouse_map.pgm     # PGM grayscale image file (same directory)
mode: trinary                # How to interpret pixel values
resolution: 0.050            # Each pixel = 0.05 m × 0.05 m
origin: [-14.528, -5.250, 0] # Map origin in metres: the real-world position of the
                             # bottom-left corner of the PGM image
negate: 0                    # 0 = dark pixels are walls (standard)
occupied_thresh: 0.65        # Pixels with p > 0.65 are marked as occupied (wall)
free_thresh: 0.196           # Pixels with p < 0.196 are marked as free (drivable)
```

### Pixel Value → Occupancy Mapping

The PGM image is greyscale (0 = black, 255 = white).  With `negate: 0`:

| Pixel value | Occupancy probability | Cell state |
|-------------|----------------------|------------|
| 0 (black) | 1.0 (definitely occupied) | Wall |
| 205 (light grey) | ≈ 0.196 | Boundary — treated as free |
| 255 (white) | 0.0 (definitely free) | Drivable space |
| In-between | unknown range | Unknown |

### Map Origin

`origin: [-14.528, -5.250, 0]` means the bottom-left corner of the map image corresponds
to the map-frame coordinates (−14.528 m, −5.250 m).  This is determined by the SLAM
Toolbox when the map was first saved.  The robot's spawn point (0.5 m, 0.5 m) is therefore
approximately 15 m to the right of and 5.75 m above the image's bottom-left corner.
