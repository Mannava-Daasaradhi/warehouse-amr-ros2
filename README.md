# Warehouse AMR — ROS 2 Jazzy + Gazebo Harmonic

Autonomous Mobile Robot (AMR) for warehouse navigation using Nav2 and SLAM Toolbox.

## Stack
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Nav2 (Navigation 2)
- SLAM Toolbox
- robot_localization (EKF)

## Prerequisites
Ubuntu 24.04 LTS with ROS 2 Jazzy installed.
```bash
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-common ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-robot-localization ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep
```

## Build
```bash
cd ~/warehouse_amr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch (SLAM mapping mode)
```bash
ros2 launch warehouse_robot_bringup warehouse_sim.launch.py slam:=true
```

## Launch (Navigation mode — after saving a map)
```bash
ros2 launch warehouse_robot_bringup warehouse_sim.launch.py slam:=false map:=/path/to/map.yaml
```

## Known Issues
- `diff_drive_controller` loading issue under investigation — contributions welcome
- See [Issues](../../issues) for current status

## Packages
| Package | Purpose |
|---|---|
| `warehouse_robot_description` | URDF/Xacro, meshes, controller config |
| `warehouse_robot_gazebo` | Gazebo world (shelves, walls, dock) |
| `warehouse_robot_navigation` | Nav2 params, EKF config, maps |
| `warehouse_robot_bringup` | Launch files |
| `warehouse_robot_navigation_py` | Python mission executor (Nav2 API) |
