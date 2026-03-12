# Launch Files Explained

> **Directory:** `src/warehouse_amr/launch/`

This document explains all three launch files, how they work individually, and how they compose into the full system.

---

## Table of Contents

1. [What Is a ROS 2 Launch File?](#1-what-is-a-ros-2-launch-file)
2. [sim.launch.py — The Simulation Launch](#2-simlaunchpy--the-simulation-launch)
3. [navigation_launch.py — The Nav2 Stack](#3-navigation_launchpy--the-nav2-stack)
4. [mission.launch.py — The Full Demo](#4-missionlaunchpy--the-full-demo)
5. [Startup Sequence and Timing](#5-startup-sequence-and-timing)
6. [Launch Arguments Reference](#6-launch-arguments-reference)

---

## 1. What Is a ROS 2 Launch File?

A ROS 2 launch file is a Python script that tells ROS 2 which nodes to start, in what
order, with what parameters, and with what inter-node connections.

Every launch file defines a function called `generate_launch_description()` that
returns a `LaunchDescription` — a list of actions to execute.

Common action types used in this project:

| Action | Purpose |
|--------|---------|
| `Node(...)` | Start a single ROS 2 node |
| `IncludeLaunchDescription(...)` | Include and execute another launch file |
| `DeclareLaunchArgument(...)` | Declare a user-configurable argument (like a CLI flag) |
| `TimerAction(period, actions)` | Wait `period` seconds then run a list of actions |
| `RegisterEventHandler(OnProcessStart(...))` | Run an action only after a specific node has started |

---

## 2. `sim.launch.py` — The Simulation Launch

> **Command:** `ros2 launch warehouse_amr sim.launch.py`

This is the main simulation launch file.  It starts **everything except the mission executor**.

### Step-by-Step Walkthrough

#### Step 1 — Resolve package path

```python
pkg_dir = get_package_share_directory(PKG)
```

`get_package_share_directory('warehouse_amr')` returns the absolute path to the installed
package's `share/` directory (e.g. `~/ros2_ws/install/warehouse_amr/share/warehouse_amr`).
All other paths (URDF, config, worlds) are built relative to this.

#### Step 2 — Read launch arguments

```python
slam     = LaunchConfiguration('slam')
run_rviz = LaunchConfiguration('rviz')
```

`LaunchConfiguration` objects are lazy references — they hold the value of a declared
argument but only resolve at runtime.  This allows conditional expressions like
`IfCondition(slam)` to be evaluated after the user has provided their values.

#### Step 3 — Process the URDF

```python
robot_description = xacro.process_file(
    xacro_file,
    mappings={'controllers_yaml': controllers_yaml}
).toxml()
```

xacro is called **directly in Python** (not via a shell command) to expand the
`warehouse_robot.urdf.xacro` macros into a plain URDF XML string.
The `controllers_yaml` path is passed in as a xacro argument so the Gazebo
ros2_control plugin knows where to find its configuration.

> **Why at launch time?**  The URDF needs to contain the absolute path to the config
> file, but absolute paths differ per machine.  Generating the URDF at launch time
> means no hardcoded paths in the source file.

#### Step 4 — Gazebo Sim

```python
gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(...'gz_sim.launch.py'...),
    launch_arguments={
        'gz_args': f'-r -v2 {world_file}',   # -r = run immediately, -v2 = verbose level 2
        'on_exit_shutdown': 'true',           # kill all ROS 2 processes when Gazebo exits
    }.items(),
)
```

Includes the standard `gz_sim.launch.py` from the `ros_gz_sim` package, which starts
the Gazebo Sim server and GUI.  The `-r` flag tells Gazebo to start the simulation
clock immediately instead of waiting for the user to press Play.

#### Step 5 — Robot State Publisher

```python
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': robot_description,   # the URDF XML string from Step 3
        'use_sim_time': True,
    }],
)
```

`robot_state_publisher` does two things:
1. Publishes the URDF on `/robot_description` (so other nodes can read it).
2. Subscribes to `/joint_states` and broadcasts dynamic TF transforms
   (e.g., the spinning wheels) on top of the static URDF transforms.

#### Step 6 — Spawn Robot (Event Handler)

```python
spawn_after_rsp = RegisterEventHandler(
    OnProcessStart(
        target_action=robot_state_publisher,
        on_start=[spawn_entity],              # run spawn_entity once RSP has started
    )
)
```

The robot can only be spawned after `robot_state_publisher` has started and published
the `/robot_description` topic.  The event handler pattern ensures correct ordering
without using a hard-coded timer.

The spawn command places the robot at:
- Position: x=0.5 m, y=0.5 m, z=0.01 m (just above the floor)
- Heading: −90° (facing −Y direction = south in the warehouse)

#### Step 7 — Gazebo ↔ ROS Bridge

```python
bridge_args = [
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',       # sim time
    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',  # LiDAR
    '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',      # IMU
    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',     # velocity commands
    '/diff_drive_controller/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry', # wheel odom
    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',          # transforms
]
```

Bridge syntax: `TOPIC@ROS_TYPE[GZ_TYPE` means **Gazebo → ROS** (one-way).
`TOPIC@ROS_TYPE]GZ_TYPE` means **ROS → Gazebo** (one-way, note the `]` direction).

`[` = Gazebo sends to ROS.  `]` = ROS sends to Gazebo.

So `/cmd_vel` flows **ROS → Gazebo** (Nav2 commands go into the simulator), while
all sensor topics flow **Gazebo → ROS** (sensor data comes out of the simulator).

#### Step 8 — Twist Stamper

```python
twist_stamper = Node(
    package='twist_stamper',
    executable='twist_stamper',
    remappings=[
        ('/cmd_vel_in',  '/cmd_vel'),                        # reads plain Twist
        ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),  # writes TwistStamped
    ],
)
```

**ros2_controllers 4.x (Jazzy) requires `TwistStamped`** for the diff_drive_controller
input, but Nav2 publishes a plain `Twist` on `/cmd_vel`.  `twist_stamper` converts
between the two message types by adding a header with the current timestamp.

#### Step 9 — Controllers (delayed 15 s)

```python
controllers_delayed = TimerAction(period=15.0, actions=[
    joint_state_broadcaster,
    diff_drive_controller,
])
```

The `spawner` nodes activate the controllers by calling a service on `/controller_manager`.
`controller_manager` only exists after Gazebo has loaded the robot model and the
`gz_ros2_control` plugin has started — which takes about 10–15 seconds.  The 15-second
delay provides a safe margin.

#### Step 10 — EKF Node

```python
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    parameters=[
        os.path.join(pkg_dir, 'config', 'ekf.yaml'),
        {'use_sim_time': True},
    ],
)
```

Starts immediately.  The EKF will wait for sensor data to arrive and will produce
`/odometry/filtered` and the `odom → base_footprint` TF as soon as both `/imu/data_raw`
and `/diff_drive_controller/odom` are publishing.

#### Step 11 — SLAM Toolbox (conditional)

```python
slam_toolbox = IncludeLaunchDescription(
    ...,
    condition=IfCondition(slam),   # only start if slam:=true (the default)
)
```

SLAM Toolbox is only started if the `slam` argument is `true`.  When `slam:=false`,
the `map_server + amcl` nodes are started instead (see Step 12).

#### Step 12 — Map Server + AMCL (conditional)

```python
map_server = Node(..., condition=UnlessCondition(slam))
amcl       = Node(..., condition=UnlessCondition(slam))
```

`UnlessCondition(slam)` means "start this node unless slam is true" — i.e., only when
`slam:=false`.  This is the localisation mode that uses a pre-built map.

#### Step 13 — Nav2 (delayed 8 s)

```python
nav2_delayed = TimerAction(period=8.0, actions=[nav2])
```

Nav2 needs SLAM to publish at least one `/map` message and the TF tree to be complete
before it can initialise.  Eight seconds is enough time for SLAM Toolbox to come up.

#### Step 14 — RViz2 (conditional)

```python
rviz2 = Node(..., condition=IfCondition(run_rviz))
```

RViz2 is started by default but can be disabled with `rviz:=false`.

---

## 3. `navigation_launch.py` — The Nav2 Stack

> **Included by:** `sim.launch.py`

This file starts all eight Nav2 server nodes **and** a lifecycle manager that brings
them up in the correct order.

### Why a Separate File?

Separating Nav2 into its own launch file makes it easy to:
- Start Nav2 independently (e.g., on a real robot without Gazebo).
- Override Nav2 parameters without touching the simulation launch.
- Delay Nav2 startup using `TimerAction` in the parent file (which is exactly what `sim.launch.py` does).

### Arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `use_sim_time` | `true` | Whether Nav2 should use `/clock` (simulation time) |
| `params_file` | (required) | Path to `nav2_params.yaml` |

### Nav2 Servers

Each server is a separate ROS 2 node with a specific role:

| Node | Role |
|------|------|
| `controller_server` | Runs the local planner (Regulated Pure Pursuit); produces `/cmd_vel_smoothed` |
| `smoother_server` | Smooths the global path to remove jagged corners |
| `planner_server` | Runs the global planner (NavFn/Dijkstra); produces `/plan` |
| `behavior_server` | Executes recovery behaviours (spin, backup, wait) |
| `bt_navigator` | Top-level action server; orchestrates everything via a Behaviour Tree |
| `waypoint_follower` | Drives through a list of poses in sequence |
| `velocity_smoother` | Applies acceleration limits to velocity commands |
| `collision_monitor` | Final safety gate; stops the robot if an obstacle is about to be hit |

### Lifecycle Manager

```python
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{
        'autostart': True,
        'node_names': ['controller_server', 'smoother_server', ...],
    }],
)
```

All Nav2 nodes are **lifecycle nodes**, meaning they have explicit states:
`unconfigured → inactive → active → finalized`.

The lifecycle manager brings all listed nodes through `configure → activate`
on startup (`autostart: True`) and cleanly shuts them down on exit.

This is important because Nav2 nodes depend on each other (e.g., the `bt_navigator`
needs the `planner_server` and `controller_server` to be active before it can
accept goals).  The lifecycle manager ensures correct ordering.

---

## 4. `mission.launch.py` — The Full Demo

> **Command:** `ros2 launch warehouse_amr mission.launch.py`

This is the simplest launch file: it includes `sim.launch.py` and then starts
the `warehouse_mission_executor` node with a 30-second delay.

```python
sim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_dir, 'launch', 'sim.launch.py')
    ),
)

mission_node = TimerAction(
    period=30.0,               # wait 30 seconds for Nav2 to fully initialise
    actions=[
        Node(
            package='warehouse_amr',
            executable='mission_executor.py',
            name='warehouse_mission_executor',
            output='screen',
            parameters=[{
                'mission_sequence': ['A1', 'B2', 'C3', 'DOCK'],  # the default route
                'auto_return_to_dock': True,
                'map_frame': 'map',
            }],
        ),
    ],
)
```

### Why 30 Seconds?

The full startup sequence takes approximately:

| Time | Event |
|------|-------|
| 0 s | Gazebo starts |
| 0–3 s | Robot spawned, bridge starts |
| 0–3 s | EKF, SLAM Toolbox start |
| 8 s | Nav2 stack starts |
| 10–12 s | SLAM publishes first map; Nav2 lifecycle activates |
| 15 s | ros2_control controllers activate (robot can move) |
| 30 s | Mission executor starts; calls `waitUntilNav2Active()` |

Thirty seconds gives the entire stack enough time to reach the active state before
the first navigation goal is sent.  The mission executor also calls
`waitUntilNav2Active()` as an extra safeguard — even if the timeout fires early,
the executor will block until Nav2 is truly ready.

### Mission Parameters

The mission executor accepts three ROS parameters:

| Parameter | Value in `mission.launch.py` | Meaning |
|-----------|------------------------------|---------|
| `mission_sequence` | `['A1', 'B2', 'C3', 'DOCK']` | Ordered list of waypoints to visit |
| `auto_return_to_dock` | `True` | If the last step isn't DOCK, append a DOCK step automatically |
| `map_frame` | `'map'` | Reference frame for all navigation goal poses |

---

## 5. Startup Sequence and Timing

The diagram below shows the full timeline when `mission.launch.py` is run:

```
t = 0 s ─── Gazebo Sim starts (warehouse.sdf)
             robot_state_publisher starts → publishes /robot_description
             → (event handler) spawn_warehouse_amr fires
             gz_ros_bridge starts (clock, scan, imu, odom, cmd_vel, tf)
             twist_stamper starts
             ekf_filter_node starts (waits for sensors)
             slam_toolbox starts (waits for /scan + TF)

t = 8 s ─── Nav2 stack starts (controller, planner, bt_navigator, etc.)
             lifecycle_manager_navigation: configure → activate

t = 10–12 s ─ SLAM builds first map → publishes /map
               Nav2 lifecycle activates fully

t = 15 s ─── joint_state_broadcaster activates
              diff_drive_controller activates → robot can now receive commands

t = 30 s ─── warehouse_mission_executor starts
              calls waitUntilNav2Active() → returns immediately (Nav2 already up)
              → robot begins navigating to A1
```

---

## 6. Launch Arguments Reference

### `sim.launch.py` Arguments

| Argument | Type | Default | Choices | Description |
|----------|------|---------|---------|-------------|
| `slam` | string | `true` | `true` / `false` | Run SLAM Toolbox (true) or AMCL with pre-built map (false) |
| `rviz` | string | `true` | `true` / `false` | Open RViz2 |

**Examples:**

```bash
# Default (SLAM + RViz2)
ros2 launch warehouse_amr sim.launch.py

# Pre-built map, no RViz2
ros2 launch warehouse_amr sim.launch.py slam:=false rviz:=false

# SLAM but no RViz2 (headless mode)
ros2 launch warehouse_amr sim.launch.py rviz:=false
```

### `navigation_launch.py` Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `use_sim_time` | string | `true` | Whether Nav2 uses simulation time |
| `params_file` | string | (none) | **Required** — absolute path to `nav2_params.yaml` |

This file is not normally called directly; it is included by `sim.launch.py`.

### `mission.launch.py` Arguments

`mission.launch.py` does not declare additional arguments of its own.
To customise the mission, either:

1. Edit `parameters=[{'mission_sequence': [...], ...}]` directly in `mission.launch.py`.
2. Run the mission executor separately with `--ros-args`:
   ```bash
   ros2 run warehouse_amr mission_executor.py --ros-args \
     -p mission_sequence:='["B1","C2","A3","DOCK"]' \
     -p auto_return_to_dock:=false
   ```
