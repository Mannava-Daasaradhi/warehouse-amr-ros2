# Warehouse AMR ROS 2 — Step-by-Step Guide

This guide walks you through every step needed to install, build, and run the
Warehouse Autonomous Mobile Robot (AMR) project from scratch on **Ubuntu 24.04 Noble**
with **ROS 2 Jazzy**.

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Install ROS 2 Jazzy](#2-install-ros-2-jazzy)
3. [Install Gazebo Harmonic](#3-install-gazebo-harmonic)
4. [Install ROS 2 Navigation and Simulation Packages](#4-install-ros-2-navigation-and-simulation-packages)
5. [Create Your Workspace and Clone the Repository](#5-create-your-workspace-and-clone-the-repository)
6. [Install Python Dependencies](#6-install-python-dependencies)
7. [Build the Package](#7-build-the-package)
8. [Source the Workspace](#8-source-the-workspace)
9. [Run the Simulation (SLAM mode — default)](#9-run-the-simulation-slam-mode--default)
10. [Watch What Happens in RViz2](#10-watch-what-happens-in-rviz2)
11. [Save the Map After SLAM](#11-save-the-map-after-slam)
12. [Run the Simulation (Pre-built Map / AMCL mode)](#12-run-the-simulation-pre-built-map--amcl-mode)
13. [Run the Full Autonomous Mission](#13-run-the-full-autonomous-mission)
14. [Monitor the Mission in Real Time](#14-monitor-the-mission-in-real-time)
15. [Customise the Waypoint Sequence](#15-customise-the-waypoint-sequence)
16. [Run the Mission Executor Standalone](#16-run-the-mission-executor-standalone)
17. [Troubleshooting](#17-troubleshooting)

---

## 1. System Requirements

| Item | Requirement |
|------|-------------|
| Operating System | Ubuntu 24.04 LTS (Noble Numbat) |
| ROS 2 version | Jazzy Jalisco |
| Gazebo version | Harmonic (gz-harmonic) |
| Python | 3.12 (comes with Ubuntu 24.04) |
| RAM | >= 8 GB recommended (Gazebo + Nav2 is memory-heavy) |
| GPU | Optional but strongly recommended for Gazebo rendering |
| Disk space | >= 5 GB free |

> **Important:** ROS 2 Jazzy is only supported on Ubuntu 24.04.
> If you are on Ubuntu 22.04, use ROS 2 Humble instead and adjust every
> package name that contains `jazzy` to `humble`.

---

## 2. Install ROS 2 Jazzy

Run these commands one by one in a terminal.

**Step 2.1 — Set the locale to UTF-8**

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Step 2.2 — Add the ROS 2 apt repository**

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Step 2.3 — Install ROS 2 Jazzy Desktop**

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

The `desktop` meta-package includes RViz2, `rqt`, `rclpy`, and all core ROS 2 tools.

**Step 2.4 — Source ROS 2 in every terminal (add to ~/.bashrc)**

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify:**

```bash
ros2 --version
# Expected: ros2, version Jazzy
```

---

## 3. Install Gazebo Harmonic

Gazebo Harmonic is the simulator paired with ROS 2 Jazzy.

```bash
sudo apt install -y gz-harmonic
```

Install the ROS ↔ Gazebo bridge packages:

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control
```

**Verify:**

```bash
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

---

## 4. Install ROS 2 Navigation and Simulation Packages

These are all the ROS 2 packages the project depends on:

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-twist-stamper \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-rviz2
```

---

## 5. Create Your Workspace and Clone the Repository

**Step 5.1 — Create the ROS 2 workspace directory**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Step 5.2 — Clone this repository into the `src` folder**

```bash
git clone https://github.com/Mannava-Daasaradhi/warehouse-amr-ros2.git .
# (the trailing dot clones into the current directory)
```

After cloning, your workspace should look like this:

```
~/ros2_ws/
└── src/
    └── warehouse_amr/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        ├── launch/
        ├── maps/
        ├── rviz/
        ├── scripts/
        ├── urdf/
        └── worlds/
```

---

## 6. Install Python Dependencies

The `mission_executor.py` script uses only packages that come with ROS 2 Jazzy,
so no extra `pip install` is required.  However, verify the Nav2 simple commander
is available:

```bash
python3 -c "from nav2_simple_commander.robot_navigator import BasicNavigator; print('OK')"
```

If you see `OK`, you are good.  If you see an `ImportError`, run:

```bash
sudo apt install -y ros-jazzy-nav2-simple-commander
```

---

## 7. Build the Package

Go to the workspace root and build with `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select warehouse_amr --symlink-install
```

- `--packages-select warehouse_amr` builds only this package (faster than building everything).
- `--symlink-install` creates symlinks instead of copies so edits to scripts and
  launch files take effect immediately without a rebuild.

Expected output:

```
Starting >>> warehouse_amr
Finished <<< warehouse_amr [1.5s]

Summary: 1 package finished [2.0s]
```

> **If you see a Python xacro error** (`ModuleNotFoundError: No module named 'xacro'`),
> run `sudo apt install -y ros-jazzy-xacro` and rebuild.

---

## 8. Source the Workspace

Every time you open a new terminal, source both ROS 2 and your workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

To make this automatic, add to `~/.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify the package is found:**

```bash
ros2 pkg list | grep warehouse_amr
# Expected: warehouse_amr
```

---

## 9. Run the Simulation (SLAM mode — default)

This is the main simulation launch.  It starts:
- Gazebo Sim (warehouse world)
- Robot spawned at position (0.5 m, 0.5 m)
- Gazebo ↔ ROS bridge (LiDAR, IMU, odometry, clock, cmd_vel)
- `robot_state_publisher` (URDF + TF tree)
- `ekf_filter_node` (wheel odom + IMU fusion)
- `slam_toolbox` (builds the map while driving)
- Full Nav2 stack (planner, controller, behaviors, etc.)
- RViz2 (pre-configured visualisation)

```bash
ros2 launch warehouse_amr sim.launch.py
```

**Optional arguments:**

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `slam` | `true` / `false` | `true` | Use SLAM Toolbox (true) or pre-built map + AMCL (false) |
| `rviz` | `true` / `false` | `true` | Open RViz2 |

Example — simulation without RViz2:

```bash
ros2 launch warehouse_amr sim.launch.py rviz:=false
```

**What you will see in the terminal:**

- Lines like `[gz_ros_bridge-1] [INFO] Creating bridge for topic /scan` — the sensor bridge is working.
- Lines like `[slam_toolbox-1] [INFO] Registering sensor…` — SLAM is active.
- After ~8 seconds: `[bt_navigator-1] Creating` — Nav2 is coming up.
- After ~15 seconds: `[spawner-1] Configured and activated diff_drive_controller` — the robot is ready to move.

---

## 10. Watch What Happens in RViz2

When RViz2 opens with `warehouse_nav.rviz`, you will see:

| Panel | What it shows |
|-------|--------------|
| **Map** (grey grid) | The occupancy map being built by SLAM Toolbox |
| **Robot model** | Orange box with two drive wheels and two casters |
| **LaserScan** | Red dots showing the 360° LiDAR hits |
| **TF frames** | Coordinate axes for `map`, `odom`, `base_link`, `lidar_link`, etc. |
| **Global Path** | The green line from the robot to the current goal |
| **Local Costmap** | Purple/grey grid around the robot showing obstacles |

To send a **manual navigation goal** from RViz2:

1. Click the **"2D Nav Goal"** button in the top toolbar.
2. Click and drag on the map to set the goal pose and heading.
3. Nav2 will plan a path and drive the robot there.

---

## 11. Save the Map After SLAM

After you have driven the robot around the warehouse to build a complete map,
save it with:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/warehouse_amr/maps/warehouse_map
```

This creates two files:
- `warehouse_map.pgm` — grayscale image of the occupancy grid
- `warehouse_map.yaml` — metadata (resolution, origin, thresholds)

These files are already committed in the repository as a starter map.

---

## 12. Run the Simulation (Pre-built Map / AMCL mode)

If you already have a saved map and want to use AMCL particle-filter localisation
instead of SLAM, pass `slam:=false`:

```bash
ros2 launch warehouse_amr sim.launch.py slam:=false
```

In this mode:
- `map_server` serves the pre-built `warehouse_map.pgm`.
- `amcl` localises the robot within that map using the LiDAR.
- SLAM Toolbox is **not** started.
- The initial pose is set automatically at (0.5, 0.5) facing −Y (see `nav2_params.yaml`).

> **When to use this:** AMCL mode is faster and uses less CPU than SLAM.
> Use it once you have a good map saved.

---

## 13. Run the Full Autonomous Mission

The full mission launch combines the simulation with the autonomous mission executor.
The mission executor waits 30 seconds for Nav2 to fully come up, then drives the
robot through the configured waypoint sequence.

```bash
ros2 launch warehouse_amr mission.launch.py
```

**Default waypoint sequence:** `A1 -> B2 -> C3 -> DOCK`

**What happens, step by step:**

| Time | Event |
|------|-------|
| t = 0 s | Gazebo starts, robot spawned |
| t = 0 s | SLAM Toolbox, EKF, Nav2 launch |
| t = 8 s | Nav2 servers become active |
| t = 15 s | ros2_control controllers activate (robot can now drive) |
| t = 30 s | `warehouse_mission_executor` starts |
| t = 30 s | Node waits for Nav2 to be ready (`waitUntilNav2Active`) |
| t ≈ 31 s | Robot starts navigating to **A1** (shelf row A, position 1) |
| — | Every 5 s: logs distance remaining + ETA |
| — | On arrival: publishes `SUCCEEDED:A1:XX.Xs` to `/amr/mission_status` |
| — | Continues to **B2**, then **C3** |
| — | After C3: auto-docks at **DOCK** (because `auto_return_to_dock: true`) |
| End | Summary printed: `3/4 succeeded | XX.Xs total` |

---

## 14. Monitor the Mission in Real Time

### Watch the status topic

Open a second terminal and run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /amr/mission_status
```

You will see one message per completed waypoint, for example:

```
data: 'SUCCEEDED:A1:28.4s'
---
data: 'SUCCEEDED:B2:35.1s'
---
data: 'SUCCEEDED:C3:41.7s'
---
data: 'SUCCEEDED:DOCK:18.2s'
```

### See all active nodes

```bash
ros2 node list
```

Expected output (with mission running):

```
/bt_navigator
/collision_monitor
/controller_server
/ekf_filter_node
/gz_ros_bridge
/lifecycle_manager_navigation
/planner_server
/robot_state_publisher
/slam_toolbox
/smoother_server
/velocity_smoother
/warehouse_mission_executor
/waypoint_follower
```

### See all active topics

```bash
ros2 topic list
```

Key topics to inspect:

```bash
ros2 topic echo /scan              # LiDAR scan data
ros2 topic echo /odometry/filtered # EKF-fused pose estimate
ros2 topic echo /map               # Live SLAM map
ros2 topic echo /cmd_vel           # Velocity commands to the robot
```

### View the TF tree

```bash
ros2 run tf2_tools view_frames
# Creates a frames.pdf showing the full transform tree
```

---

## 15. Customise the Waypoint Sequence

### Option A — Change the default in `mission.launch.py`

Edit `src/warehouse_amr/launch/mission.launch.py`, line 40:

```python
'mission_sequence': ['A1', 'B2', 'C3', 'DOCK'],
```

Change it to any combination of these keys:

```
A1  A2  A3
B1  B2  B3
C1  C2  C3
STAGING
DOCK
```

Then rebuild:

```bash
colcon build --packages-select warehouse_amr --symlink-install
```

### Option B — Override at launch time (no rebuild needed)

```bash
ros2 launch warehouse_amr mission.launch.py
# Then in a second terminal, run the executor with custom params:
ros2 run warehouse_amr mission_executor.py --ros-args \
  -p mission_sequence:='["B1","C2","A3","DOCK"]' \
  -p auto_return_to_dock:=false
```

### Option C — Add a new shelf location

Edit `SHELF_REGISTRY` in `scripts/mission_executor.py`.
Each entry needs a name, X (metres), Y (metres), and yaw (radians):

```python
"D1": Waypoint("D1", x=11.0, y=3.5, yaw=0.0),
```

Then rebuild with `colcon build --packages-select warehouse_amr --symlink-install`.

---

## 16. Run the Mission Executor Standalone

If the simulation is already running in another terminal, you can run the
mission executor by itself:

```bash
ros2 run warehouse_amr mission_executor.py
```

Or with custom parameters:

```bash
ros2 run warehouse_amr mission_executor.py --ros-args \
  -p mission_sequence:='["A2","B1","STAGING","DOCK"]' \
  -p auto_return_to_dock:=false \
  -p map_frame:=map
```

The executor will wait until Nav2 is active before sending the first goal.

---

## 17. Troubleshooting

### ❌ `colcon build` fails with `Could not find package 'ros_gz_sim'`

You are missing the Gazebo bridge. Run:

```bash
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

---

### ❌ Robot does not appear in Gazebo

The spawn happens via an event handler that waits for `robot_state_publisher`
to start.  If `robot_state_publisher` crashes (usually a bad URDF), check:

```bash
ros2 topic echo /robot_description --once
```

If empty, check the `xacro` processing:

```bash
xacro ~/ros2_ws/src/warehouse_amr/urdf/warehouse_robot.urdf.xacro \
  controllers_yaml:=~/ros2_ws/install/warehouse_amr/share/warehouse_amr/config/ros2_controllers.yaml
```

---

### ❌ `diff_drive_controller` never activates

The controller spawner runs 15 seconds after launch.  Wait.
If it still fails, check that Gazebo has the `gz_ros2_control` plugin:

```bash
ros2 node list | grep controller_manager
# Should show: /controller_manager
```

---

### ❌ Nav2 action server not available / mission executor hangs on `waitUntilNav2Active`

Nav2 comes up ~8 seconds after launch.  The mission executor is delayed 30 seconds
(in `mission.launch.py`) precisely to avoid this.  If it still hangs:

1. Check that all Nav2 nodes are running:
   ```bash
   ros2 node list | grep -E "(bt_navigator|planner_server|controller_server|behavior_server)"
   ```
   You should see at least those four nodes listed.
2. Check `lifecycle_manager_navigation` is active:
   ```bash
   ros2 node info /lifecycle_manager_navigation
   ```
3. Increase the delay in `mission.launch.py` from `period=30.0` to `period=45.0` and rebuild.

---

### ❌ Robot does not move toward the goal (path planning fails)

The global costmap needs a map.  If SLAM has not built one yet, Nav2 cannot plan.
Wait until the grey map appears in RViz2, then try again.

Also verify the EKF is publishing a filtered odometry:

```bash
ros2 topic hz /odometry/filtered
# Should be ~30 Hz
```

---

### ❌ `[ERROR] Unknown waypoint 'XYZ'`

The waypoint name you passed is not in `SHELF_REGISTRY`.
Valid names are: `A1 A2 A3 B1 B2 B3 C1 C2 C3 STAGING DOCK`.
Names are case-insensitive (the executor upper-cases them automatically).

---

### ❌ RViz2 shows `No transform from [map] to [base_link]`

The `map → odom` transform is published by SLAM Toolbox (or AMCL).
This is normal for the first few seconds while SLAM initialises.
If it persists after 30 seconds, check:

```bash
ros2 topic hz /map
# Should publish at ~0.2 Hz (every 5 s) once SLAM has data
```

---

### 🔧 Useful debug commands

```bash
# Print the full TF tree
ros2 run tf2_tools view_frames

# Show transform between two frames
ros2 run tf2_ros tf2_echo map base_link

# Echo Nav2 plan
ros2 topic echo /plan --once

# List all active services
ros2 service list | grep nav2

# Check EKF diagnostics
ros2 topic echo /diagnostics --once
```

---

## Quick-Start Cheat Sheet

```bash
# One-time setup (only needed once per machine)
sudo apt install -y ros-jazzy-desktop gz-harmonic
sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control ros-jazzy-twist-stamper ros-jazzy-ros2-controllers \
  ros-jazzy-nav2-simple-commander ros-jazzy-xacro

# Build (only needed after code changes)
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select warehouse_amr --symlink-install
source install/setup.bash

# ── OPTION A: Simulation only (SLAM, manual goals via RViz2) ──
ros2 launch warehouse_amr sim.launch.py

# ── OPTION B: Full autonomous mission ──
ros2 launch warehouse_amr mission.launch.py

# ── In a second terminal: watch mission status ──
source ~/ros2_ws/install/setup.bash
ros2 topic echo /amr/mission_status
```
