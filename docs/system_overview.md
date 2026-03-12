# Warehouse AMR — System Overview: Nodes, Topics & TF Frames

> **ROS 2 distribution:** Jazzy  
> **Simulator:** Gazebo (Harmonic)  
> **Navigation:** Nav2 1.3+  

This document lists every ROS 2 node, topic, and TF coordinate frame that is active when the system is launched with:

```bash
ros2 launch warehouse_amr mission.launch.py
# or
ros2 launch warehouse_amr sim.launch.py slam:=true rviz:=true
```

---

## 1. Nodes

### 1.1 Simulation Layer

| Node Name | Package / Executable | What It Does |
|-----------|---------------------|--------------|
| `robot_state_publisher` | `robot_state_publisher` | Parses the URDF/xacro, publishes `/robot_description`, and broadcasts all **fixed** TF transforms from the robot's kinematic tree |
| `spawn_warehouse_amr` | `ros_gz_sim create` | One-shot node that spawns the robot model into the running Gazebo world at startup (x=0.5, y=0.5, z=0.01, yaw=−90°) |
| `gz_ros_bridge` | `ros_gz_bridge parameter_bridge` | Bidirectional bridge between Gazebo topics and ROS 2 topics (clock, lidar, IMU, cmd_vel, odom, tf) |
| `controller_manager` | `gz_ros2_control` (embedded Gazebo plugin) | Loads and manages ros2_control controllers; runs at 100 Hz |
| `joint_state_broadcaster` | `controller_manager spawner` | Reads joint encoders and publishes `/joint_states` |
| `diff_drive_controller` | `controller_manager spawner` | Reads `/diff_drive_controller/cmd_vel` (TwistStamped), drives the wheels, publishes `/diff_drive_controller/odom` |
| `twist_stamper` | `twist_stamper` | Converts `/cmd_vel` (Twist) to `/diff_drive_controller/cmd_vel` (TwistStamped) — required for ros2_controllers 4.x on Jazzy |

### 1.2 Localisation Layer

| Node Name | Package / Executable | What It Does |
|-----------|---------------------|--------------|
| `ekf_filter_node` | `robot_localization ekf_node` | Fuses wheel odometry (`/diff_drive_controller/odom`) and IMU (`/imu/data_raw`) into a smooth filtered odometry estimate; publishes `/odometry/filtered` and the `odom → base_footprint` TF transform |

**SLAM mode (default, `slam:=true`)**

| Node Name | Package / Executable | What It Does |
|-----------|---------------------|--------------|
| `slam_toolbox` (async) | `slam_toolbox online_async_launch.py` | Builds an occupancy grid map in real time from `/scan`; publishes `/map` and the `map → odom` TF transform |

**Localisation-only mode (`slam:=false`)**

| Node Name | Package / Executable | What It Does |
|-----------|---------------------|--------------|
| `map_server` | `nav2_map_server` | Loads the pre-built `warehouse_map.yaml` and publishes it on `/map` |
| `amcl` | `nav2_amcl` | Particle-filter localisation using `/scan` and `/map`; publishes the `map → odom` TF transform |
| `lifecycle_manager_localization` | `nav2_lifecycle_manager` | Manages the lifecycle of `map_server` and `amcl` (configure → activate → deactivate) |

### 1.3 Navigation Layer (Nav2)

All Nav2 nodes are started by `navigation_launch.py` and managed by a single lifecycle manager.

| Node Name | Package / Executable | What It Does |
|-----------|---------------------|--------------|
| `controller_server` | `nav2_controller` | Runs the **Regulated Pure Pursuit** local planner; converts the global path into `/cmd_vel_smoothed` at 20 Hz |
| `smoother_server` | `nav2_smoother` | Post-processes global paths with a simple smoother |
| `planner_server` | `nav2_planner` | Runs the **NavFn** (Dijkstra / A*) global planner; produces a `/plan` path on the global costmap |
| `behavior_server` | `nav2_behaviors` | Executes recovery behaviours: spin, backup, drive-on-heading, wait, assisted-teleop |
| `bt_navigator` | `nav2_bt_navigator` | Orchestrates the full navigate-to-pose pipeline using a Behaviour Tree; the main action server that `BasicNavigator.goToPose()` talks to |
| `waypoint_follower` | `nav2_waypoint_follower` | Accepts a list of waypoints and drives through them sequentially (used when calling `followWaypoints`) |
| `velocity_smoother` | `nav2_velocity_smoother` | Low-pass filters velocity commands to respect acceleration limits before forwarding to `collision_monitor` |
| `collision_monitor` | `nav2_collision_monitor` | Final safety gate: reads `/scan` and `/cmd_vel_smoothed`, slows or stops the robot if an obstacle is too close |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | Brings all Nav2 servers through their lifecycle states in the correct order |

### 1.4 Visualisation

| Node Name | Package / Executable | What It Does |
|-----------|---------------------|--------------|
| `rviz2` | `rviz2` | Displays the map, robot model, laser scan, costmaps, and planned paths (conditional on `rviz:=true`) |

### 1.5 Application Layer

| Node Name | Package / Executable | What It Does |
|-----------|---------------------|--------------|
| `warehouse_mission_executor` | `warehouse_amr mission_executor.py` | Reads a waypoint sequence from ROS parameters, sends each pose to Nav2 via `BasicNavigator.goToPose()`, monitors progress, and publishes status strings on `/amr/mission_status` |

---

## 2. Topics

### 2.1 Sensor Topics

| Topic | Type | Publisher | Subscribers | Description |
|-------|------|-----------|-------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo LiDAR plugin → `gz_ros_bridge` | `slam_toolbox` / `amcl`, `controller_server` (local costmap), `collision_monitor` | 360° LiDAR scan at 10 Hz, 720 rays, 0.1–12 m range |
| `/imu/data_raw` | `sensor_msgs/Imu` | Gazebo IMU plugin → `gz_ros_bridge` | `ekf_filter_node` | Raw IMU data at 100 Hz (angular velocity + linear acceleration) |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `robot_state_publisher` | Wheel joint positions and velocities |

### 2.2 Odometry Topics

| Topic | Type | Publisher | Subscribers | Description |
|-------|------|-----------|-------------|-------------|
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | `gz_ros_bridge` (bridged from Gazebo) | `ekf_filter_node` | Raw wheel-encoder odometry from the differential drive controller |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_filter_node` | `bt_navigator`, `velocity_smoother` | EKF-fused odometry (odom + IMU); the authoritative odometry topic used by Nav2 |

### 2.3 Velocity / Control Topics

| Topic | Type | Publisher | Subscribers | Description |
|-------|------|-----------|-------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `collision_monitor` (output) | `twist_stamper` | Final velocity command; output of collision monitor after safety check |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | `twist_stamper` | `diff_drive_controller` (Gazebo) | Stamped velocity command forwarded to the hardware interface |
| `/cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | `velocity_smoother` | `collision_monitor` | Acceleration-limited velocity; input to collision monitor |

### 2.4 Map & Costmap Topics

| Topic | Type | Publisher | Subscribers | Description |
|-------|------|-----------|-------------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` or `map_server` | `planner_server` (global costmap static layer), `amcl` | 2-D occupancy grid of the warehouse, resolution 0.05 m |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | `planner_server` | `rviz2` | Combined global costmap (static + obstacle + inflation layers) |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | `controller_server` | `rviz2` | Rolling 5×5 m local costmap (voxel + inflation layers) |
| `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` | `controller_server` | `collision_monitor`, `behavior_server` | Robot footprint polygon used for collision checking |

### 2.5 Navigation Topics

| Topic | Type | Publisher | Subscribers | Description |
|-------|------|-----------|-------------|-------------|
| `/plan` | `nav_msgs/Path` | `planner_server` | `controller_server`, `rviz2` | Global path from current pose to goal |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | `spawn_warehouse_amr` (at startup) | URDF XML string of the robot model |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 (interactive) | `bt_navigator` | Manual navigation goal set from RViz2 |

### 2.6 TF Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/tf` | `tf2_msgs/TFMessage` | `gz_ros_bridge`, `ekf_filter_node`, `slam_toolbox`/`amcl`, `robot_state_publisher` | Dynamic TF transforms broadcast at runtime |
| `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` | Static TF transforms from the URDF (fixed joints) |

### 2.7 Simulation Topics

| Topic | Type | Publisher | Subscribers | Description |
|-------|------|-----------|-------------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → `gz_ros_bridge` | All nodes using `use_sim_time: true` | Simulation clock at real-time or faster-than-real-time rate |

### 2.8 Application Topics

| Topic | Type | Publisher | Subscribers | Description |
|-------|------|-----------|-------------|-------------|
| `/amr/mission_status` | `std_msgs/String` | `warehouse_mission_executor` | Any monitoring node / dashboard | Status string after every navigation step, format: `STATUS:WAYPOINT:TIMEs[:detail]` |

---

## 3. TF Coordinate Frames

The TF tree forms a chain from the global map frame down to each sensor frame.

```
map
 └── odom                          (published by: slam_toolbox or amcl)
      └── base_footprint            (published by: ekf_filter_node)
           └── base_link            (published by: robot_state_publisher — fixed joint)
                ├── left_wheel_link  (published by: robot_state_publisher — continuous joint)
                ├── right_wheel_link (published by: robot_state_publisher — continuous joint)
                ├── front_caster_link(published by: robot_state_publisher — fixed joint)
                ├── rear_caster_link (published by: robot_state_publisher — fixed joint)
                ├── lidar_link       (published by: robot_state_publisher — fixed joint)
                └── imu_link         (published by: robot_state_publisher — fixed joint)
```

### 3.1 Frame Descriptions

| Frame | Parent | Transform Source | Description |
|-------|--------|-----------------|-------------|
| `map` | — (root) | — | Global fixed frame; the map's coordinate system |
| `odom` | `map` | `slam_toolbox` (SLAM mode) or `amcl` (localisation mode) | Continuous odometry frame; drift-free in the short term, corrected by SLAM/AMCL over time |
| `base_footprint` | `odom` | `ekf_filter_node` | Robot's 2-D footprint projected on the ground plane; used by Nav2 as the planning reference frame |
| `base_link` | `base_footprint` | `robot_state_publisher` (URDF fixed joint) | Centre of the robot chassis, lifted to wheel-axle height (z = wheel_radius = 0.10 m) |
| `left_wheel_link` | `base_link` | `robot_state_publisher` (joint state from encoder) | Left drive wheel; offset +0.26 m in Y from base_link |
| `right_wheel_link` | `base_link` | `robot_state_publisher` (joint state from encoder) | Right drive wheel; offset −0.26 m in Y from base_link |
| `front_caster_link` | `base_link` | `robot_state_publisher` (URDF fixed joint) | Front passive caster; offset forward at x = base_length/2 − caster_radius |
| `rear_caster_link` | `base_link` | `robot_state_publisher` (URDF fixed joint) | Rear passive caster; offset backward at x = −base_length/2 + caster_radius |
| `lidar_link` | `base_link` | `robot_state_publisher` (URDF fixed joint) | LiDAR sensor origin; mounted forward and on top of the chassis |
| `imu_link` | `base_link` | `robot_state_publisher` (URDF fixed joint) | IMU sensor origin; mounted at the geometric centre of the chassis |

### 3.2 Key Frame Parameters (from URDF)

| Parameter | Value |
|-----------|-------|
| Base chassis (L × W × H) | 0.60 m × 0.50 m × 0.25 m |
| Drive wheel radius | 0.10 m |
| Drive wheel separation | 0.52 m |
| `base_footprint` → `base_link` offset | z = +0.10 m (wheel radius) |
| `base_link` → `lidar_link` offset | x = base_length/2 − 0.07, z = base_height + 0.035 |
| `base_link` → `imu_link` offset | z = base_height/2 = 0.125 m |

---

## 4. System Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                            Gazebo Simulation                                 │
│  ┌────────────────┐  /scan (LaserScan)    ┌──────────────────────────────┐   │
│  │  LiDAR plugin  │─────────────────────► │                              │   │
│  └────────────────┘                       │       gz_ros_bridge          │   │
│  ┌────────────────┐  /imu/data_raw (Imu)  │  (Gazebo ↔ ROS 2 bridge)    │   │
│  │  IMU plugin    │─────────────────────► │                              │   │
│  └────────────────┘                       │  /clock, /tf, /odom, /scan  │   │
│  ┌────────────────┐  /diff_drive/odom     │  /imu/data_raw, /cmd_vel    │   │
│  │  DiffDrive ctrl│◄────────────────────► │                              │   │
│  └────────────────┘  /cmd_vel (stamped)   └──────────────┬───────────────┘   │
└─────────────────────────────────────────────────────────┼────────────────────┘
                                                           │ ROS 2
          ┌────────────────────────────────────────────────┼─────────────────────┐
          │                                                ▼                      │
          │  ┌─────────────────┐         ┌───────────────────────────────────┐   │
          │  │robot_state_pub  │         │          ekf_filter_node           │   │
          │  │ /robot_description         │  odom + IMU → /odometry/filtered  │   │
          │  │ /tf_static      │         │  TF: odom → base_footprint        │   │
          │  └─────────────────┘         └─────────────────┬─────────────────┘   │
          │                                                 │                     │
          │  ┌─────────────────┐                           │                     │
          │  │  slam_toolbox   │◄──── /scan ───────────────┘                     │
          │  │  /map           │    TF: map → odom                               │
          │  └─────────────────┘                                                 │
          │                                                                       │
          │  ┌─────────────────────────────────────────────────────────────────┐ │
          │  │                     Nav2 Stack                                  │ │
          │  │  planner_server ──► controller_server ──► velocity_smoother     │ │
          │  │        ▲                  ▲                       │             │ │
          │  │  global_costmap    local_costmap          collision_monitor      │ │
          │  │        ▲                  ▲                       │             │ │
          │  │       /map              /scan              /cmd_vel              │ │
          │  │                                                   │             │ │
          │  │  bt_navigator ◄── goToPose() action               │             │ │
          │  └─────────────────────────────────────────────────────────────────┘ │
          │                  ▲                                   │               │
          │                  │                                   ▼               │
          │  ┌───────────────────────────┐       ┌─────────────────────────┐    │
          │  │ warehouse_mission_executor │       │     twist_stamper        │    │
          │  │  publishes /amr/mission_  │       │ /cmd_vel → TwistStamped │    │
          │  │  status after each step   │       └─────────────────────────┘    │
          │  └───────────────────────────┘                                       │
          └───────────────────────────────────────────────────────────────────────┘
```

---

## 5. Launch File Sequence

When `mission.launch.py` is run, the following startup sequence occurs:

| Time | Event |
|------|-------|
| 0 s | Gazebo loads `warehouse.sdf` world |
| 0 s | `robot_state_publisher` starts; publishes `/robot_description`, static TFs, and (via event handler) triggers `spawn_warehouse_amr` |
| 0 s | `gz_ros_bridge` starts bridging topics |
| 0 s | `twist_stamper` starts |
| 0 s | `ekf_filter_node` starts |
| 0 s | `slam_toolbox` starts (or `map_server + amcl` if `slam:=false`) |
| 8 s | Nav2 stack starts (delayed to let SLAM come up first) |
| 15 s | `joint_state_broadcaster` and `diff_drive_controller` spawned (Gazebo needs time to init) |
| 30 s | `warehouse_mission_executor` starts and begins navigating |

---

## 6. ROS Parameters Summary

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| `warehouse_mission_executor` | `mission_sequence` | `['A1','B2','DOCK']` | Ordered list of waypoints |
| `warehouse_mission_executor` | `auto_return_to_dock` | `true` | Auto-append DOCK at end |
| `warehouse_mission_executor` | `map_frame` | `'map'` | Reference frame for goal poses |
| `ekf_filter_node` | `frequency` | `30.0` | Filter update rate (Hz) |
| `ekf_filter_node` | `two_d_mode` | `true` | Constrain to XY plane |
| `diff_drive_controller` | `wheel_separation` | `0.52` | Distance between drive wheels (m) |
| `diff_drive_controller` | `wheel_radius` | `0.10` | Drive wheel radius (m) |
| `controller_server` | `controller_frequency` | `20.0` | Local planner rate (Hz) |
| `planner_server` | `expected_planner_frequency` | `20.0` | Global planner rate (Hz) |
| `slam_toolbox` | `resolution` | `0.05` | Map cell size (m) |
| `slam_toolbox` | `max_laser_range` | `12.0` | Max LiDAR range used for mapping (m) |
