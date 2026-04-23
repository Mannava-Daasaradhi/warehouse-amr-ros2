# Warehouse AMR — ROS 2 Autonomous Mobile Robot

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-FF6600)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-4CAF50)](LICENSE)
[![Build](https://github.com/Mannava-Daasaradhi/warehouse-amr-ros2/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/Mannava-Daasaradhi/warehouse-amr-ros2/actions/workflows/ros2-build.yml)

A complete ROS 2 simulation of a **warehouse Autonomous Mobile Robot (AMR)** that can
autonomously navigate to named shelf locations and a charging dock.

Built on **ROS 2 Jazzy**, **Gazebo Harmonic**, **Nav2**, **SLAM Toolbox**, and **robot_localization (EKF)**.

> 📸 *Demo GIF coming soon — run `ros2 launch warehouse_amr mission.launch.py` to see it live.*


## What This Project Does

```
┌──────────────────────────────────────────────────────────────────┐
│                       Warehouse Simulation                        │
│                                                                   │
│  Gazebo (3-D physics) ──► ROS bridge ──► ROS 2 topics            │
│         │                                    │                    │
│    robot URDF                          LiDAR / IMU / odom        │
│    diff-drive                                │                    │
│    LiDAR + IMU                         EKF filter node           │
│                                              │                    │
│                                        SLAM Toolbox / AMCL       │
│                                              │                    │
│                                         Nav2 stack               │
│                                   (plan → control → drive)       │
│                                              │                    │
│                                  mission_executor.py             │
│                                  (A1 → B2 → C3 → DOCK)          │
└──────────────────────────────────────────────────────────────────┘
```

The robot visits a configurable list of shelf positions (e.g. `A1 → B2 → C3 → DOCK`)
one at a time, using Nav2 to plan and execute collision-free paths.
After every step it publishes a status string so external monitoring tools can track progress.

---

## System Requirements

| Item | Requirement |
|------|-------------|
| OS | Ubuntu 24.04 LTS (Noble Numbat) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-harmonic) |
| Python | 3.12 |
| RAM | ≥ 8 GB |
| Disk | ≥ 5 GB free |

---

## Quick Start

```bash
# ── 1. Clone ──────────────────────────────────────────────────────
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Mannava-Daasaradhi/warehouse-amr-ros2.git .

# ── 2. Install dependencies ───────────────────────────────────────
sudo apt install -y ros-jazzy-desktop gz-harmonic \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control ros-jazzy-twist-stamper \
  ros-jazzy-ros2-controllers ros-jazzy-xacro

# ── 3. Build ──────────────────────────────────────────────────────
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select warehouse_amr --symlink-install
source install/setup.bash

# ── 4. Run the full autonomous mission ───────────────────────────
ros2 launch warehouse_amr mission.launch.py
```

In a second terminal, watch the mission status:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /amr/mission_status
```

Expected output after ~35 seconds:
```
data: SUCCEEDED:A1:18.4s
data: SUCCEEDED:B2:14.1s
data: SUCCEEDED:C3:13.0s
data: SUCCEEDED:DOCK:28.7s
```

---

## Repository Structure

```
warehouse-amr-ros2/
├── README.md                          ← You are here
├── LICENSE                            ← Apache 2.0
├── CONTRIBUTING.md                    ← How to add waypoints, tune params, run tests
├── .github/
│   └── workflows/
│       └── ros2-build.yml             ← CI: build + test on every push
├── docs/
│   ├── step_by_step_guide.md          ← Full install & run walkthrough
│   ├── system_overview.md             ← Every node, topic, and TF frame
│   ├── mission_executor_explained.md  ← Deep-dive into mission_executor.py
│   ├── robot_description_explained.md ← URDF/XACRO robot model explained
│   ├── config_files_explained.md      ← All YAML config parameters explained
│   └── launch_files_explained.md      ← All launch files explained
└── src/
    └── warehouse_amr/                 ← The ROS 2 package
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   ├── ekf.yaml               ← robot_localization EKF parameters
        │   ├── nav2_params.yaml       ← Nav2 planner/controller/costmap params
        │   └── ros2_controllers.yaml  ← diff_drive_controller parameters
        ├── launch/
        │   ├── sim.launch.py          ← Gazebo + SLAM + Nav2 + RViz2
        │   ├── navigation_launch.py   ← All Nav2 servers
        │   └── mission.launch.py      ← sim + mission executor (full demo)
        ├── maps/
        │   ├── warehouse_map.pgm      ← Pre-built occupancy grid image
        │   └── warehouse_map.yaml     ← Map metadata (resolution, origin)
        ├── rviz/
        │   └── warehouse_nav.rviz     ← Pre-configured RViz2 layout
        ├── scripts/
        │   └── mission_executor.py    ← Autonomous mission controller node
        ├── test/
        │   └── test_mission_executor.py ← 20 unit tests (no ROS runtime needed)
        ├── urdf/
        │   └── warehouse_robot.urdf.xacro  ← Robot model (URDF + plugins)
        └── worlds/
            └── warehouse.sdf          ← Gazebo warehouse environment

---

## Documentation

| Document | What It Covers |
|----------|---------------|
| [step_by_step_guide.md](docs/step_by_step_guide.md) | Complete install, build, and run guide for total beginners |
| [system_overview.md](docs/system_overview.md) | All nodes, topics, and TF frames; full architecture diagram |
| [mission_executor_explained.md](docs/mission_executor_explained.md) | Line-by-line walkthrough of the mission controller node |
| [robot_description_explained.md](docs/robot_description_explained.md) | URDF structure, physical dimensions, sensor setup, Gazebo plugins |
| [config_files_explained.md](docs/config_files_explained.md) | Every YAML parameter in EKF, Nav2, and ros2_controllers configs |
| [launch_files_explained.md](docs/launch_files_explained.md) | How all three launch files work and how they compose |

---

## Launch Files

| Launch File | What It Starts |
|-------------|---------------|
| `sim.launch.py` | Gazebo + URDF + bridge + EKF + SLAM/AMCL + Nav2 + RViz2 |
| `navigation_launch.py` | All 8 Nav2 servers + lifecycle manager (included by `sim.launch.py`) |
| `mission.launch.py` | Everything in `sim.launch.py` **plus** the autonomous mission executor |

```bash
# Simulation with SLAM (manual navigation from RViz2)
ros2 launch warehouse_amr sim.launch.py

# Simulation with pre-built map (AMCL localisation)
ros2 launch warehouse_amr sim.launch.py slam:=false

# Full autonomous mission
ros2 launch warehouse_amr mission.launch.py
```

---

## Customising the Mission

The robot visits waypoints by name. Default sequence: **A1 → B2 → C3 → DOCK**.

Available waypoints:
```
A1  A2  A3    (Aisle A, bays 1–3, x = 2.0 m, face +X)
B1  B2  B3    (Aisle B, bays 1–3, x = 5.0 m, face -X)
C1  C2  C3    (Aisle C, bays 1–3, x = 8.0 m, face +X)
STAGING       (near origin, x=1.0 y=1.0)
DOCK          (charging dock, x=0.5 y=0.5, face -Y)
```

Override the sequence at launch time (no rebuild needed):
```bash
ros2 run warehouse_amr mission_executor.py --ros-args \
  -p mission_sequence:='["B1","C2","A3","DOCK"]' \
  -p auto_return_to_dock:=false
```

---

## How It Works (Condensed)

1. **Gazebo** simulates the warehouse world and the robot's physics.
2. **gz_ros_bridge** connects Gazebo topics (LiDAR, IMU, odometry, cmd_vel) to ROS 2.
3. **robot_localization (EKF)** fuses wheel odometry + IMU into a smooth pose estimate.
4. **SLAM Toolbox** builds a 2-D occupancy map from the LiDAR and publishes the `map → odom` TF transform.
5. **Nav2** uses that map and the EKF pose to plan collision-free paths and drive the robot.
6. **mission_executor.py** sends one Nav2 goal at a time, waits for completion, and publishes status.

For a deep dive into any component, start with the corresponding document in `docs/`.

---

## Running Tests

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select warehouse_amr
colcon test-result --verbose
```

Expected output: **20 tests, 0 errors, 0 failures, 0 skipped.**
Tests cover `_yaw_to_quat`, `_make_pose`, and all `SHELF_REGISTRY` entries.
No running ROS 2 instance or Gazebo required.

---

## Troubleshooting

See [docs/step_by_step_guide.md — Troubleshooting section](docs/step_by_step_guide.md#17-troubleshooting) for solutions to the most common problems.
