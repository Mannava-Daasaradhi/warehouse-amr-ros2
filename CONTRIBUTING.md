# Contributing to warehouse-amr-ros2

Thank you for your interest in contributing!

---

## How to add a new waypoint

1. Open `src/warehouse_amr/scripts/mission_executor.py`
2. Add an entry to `SHELF_REGISTRY`:
```python
   "D1": Waypoint("D1", x=11.0, y=3.5, yaw=0.0),
```
3. Open `src/warehouse_amr/worlds/warehouse.sdf` and add a matching shelf model and floor marker at the same coordinates.
4. Optionally update the default `mission_sequence` in `launch/mission.launch.py`.
5. Update the waypoint reference table in `README.md`.
6. Add a test to `src/warehouse_amr/test/test_mission_executor.py` verifying the new waypoint's coordinates and heading.

---

## How to tune Nav2 parameters

See `docs/config_files_explained.md` for a full explanation of every parameter.
The most commonly tuned values are:

| Parameter | File | Effect |
|-----------|------|--------|
| `desired_linear_vel` | `nav2_params.yaml` → `FollowPath` | Robot cruise speed (m/s) |
| `xy_goal_tolerance` | `nav2_params.yaml` → `general_goal_checker` | Arrival radius (m) |
| `inflation_radius` | `nav2_params.yaml` → costmaps | Obstacle clearance distance (m) |
| `nav_timeout_s` | launch-time ROS param | Max seconds per waypoint before cancelling |

---

## How to run tests

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select warehouse_amr --symlink-install
source install/setup.bash
colcon test --packages-select warehouse_amr
colcon test-result --verbose
```

All tests must pass (0 failures, 0 errors) before submitting a pull request.

---

## How to customise the mission at runtime

```bash
# Custom waypoint sequence — no rebuild needed
ros2 run warehouse_amr mission_executor.py --ros-args \
  -p mission_sequence:='["B1","C2","A3","DOCK"]' \
  -p nav_timeout_s:=60.0 \
  -p auto_return_to_dock:=false
```

---

## Pull request checklist

- [ ] `colcon build` succeeds with no warnings
- [ ] `colcon test` passes with 0 failures
- [ ] New waypoints have a matching entry in both `SHELF_REGISTRY` and `warehouse.sdf`
- [ ] Any new parameters are declared with `declare_parameter()` and documented in `docs/`
- [ ] README updated if user-facing behaviour changes