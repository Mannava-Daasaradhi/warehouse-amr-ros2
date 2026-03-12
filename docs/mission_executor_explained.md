# mission_executor.py — How It Works

> **File:** `src/warehouse_amr/scripts/mission_executor.py`  
> **Purpose:** Drive the Warehouse AMR through a configurable sequence of named waypoints using the Nav2 navigation stack.

---

## 1. Overview

`mission_executor.py` is a standalone ROS 2 node that acts as the **mission brain** of the Warehouse AMR.  
It reads a list of shelf/dock names from a ROS parameter, converts each name into a map-frame goal pose, and hands those goals to **Nav2** one at a time.  
After every step it publishes a short status string so other nodes (dashboards, fleet managers) can track progress.

```
┌─────────────────────────────────────────────────────────────────┐
│                        mission_executor.py                      │
│                                                                 │
│  ROS params ──► MissionExecutorNode.__init__()                  │
│                          │                                      │
│                          ▼                                      │
│              run_sequence()  ◄── called once from main()        │
│                  │                                              │
│                  ▼                                              │
│  for each step ──► _go(name)                                    │
│                          │                                      │
│                          ├─► _make_pose()  ──► Nav2.goToPose()  │
│                          │                                      │
│                          └─► polling loop  ──► getFeedback()    │
│                                                                 │
│  after each step ──► _pub_status()  ──► /amr/mission_status     │
│                                                                 │
│  after all steps ──► _print_summary()                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Data Types

### `MissionStatus` (Enum)

| Value | Meaning |
|-------|---------|
| `IDLE` | Node is alive, no navigation has started yet |
| `NAVIGATING` | A `goToPose` goal is currently in flight |
| `SUCCEEDED` | Nav2 confirmed the robot reached the goal |
| `FAILED` | Nav2 reported an unrecoverable failure |
| `CANCELLED` | Navigation was stopped (timeout or Ctrl-C) |

### `Waypoint` (dataclass)

Stores a named pose: `name`, `x` (metres), `y` (metres), `yaw` (radians).  
All coordinates are in the **map** frame.

### `MissionReport` (dataclass)

One record per completed navigation step: which waypoint, what outcome, how long it took, and an optional detail message.

---

## 3. The Waypoint Registry (`SHELF_REGISTRY`)

A module-level `dict` maps short names to `Waypoint` objects:

| Name | x (m) | y (m) | Yaw | Description |
|------|--------|--------|-----|-------------|
| A1 | 2.0 | 3.5 | 0° | Aisle A, bay 1 (faces +X) |
| A2 | 2.0 | 5.0 | 0° | Aisle A, bay 2 |
| A3 | 2.0 | 6.5 | 0° | Aisle A, bay 3 |
| B1 | 5.0 | 3.5 | 180° | Aisle B, bay 1 (faces −X) |
| B2 | 5.0 | 5.0 | 180° | Aisle B, bay 2 |
| B3 | 5.0 | 6.5 | 180° | Aisle B, bay 3 |
| C1 | 8.0 | 3.5 | 0° | Aisle C, bay 1 |
| C2 | 8.0 | 5.0 | 0° | Aisle C, bay 2 |
| C3 | 8.0 | 6.5 | 0° | Aisle C, bay 3 |
| STAGING | 1.0 | 1.0 | 0° | Staging area near origin |
| DOCK | 0.5 | 0.5 | −90° | Charging dock (faces −Y) |

To add new locations, insert a new `Waypoint(...)` entry in the dict.

---

## 4. Helper Functions

### `_yaw_to_quat(yaw)`

Converts a 2-D heading angle (radians) to a quaternion `(x, y, z, w)`.  
Because the robot only rotates about the vertical Z-axis the formula simplifies to:

```
x = 0,  y = 0,  z = sin(yaw/2),  w = cos(yaw/2)
```

### `_make_pose(wp, frame_id)`

Builds a `PoseStamped` ROS message from a `Waypoint`:

1. Sets `header.frame_id` to the map frame name.
2. Copies `wp.x` and `wp.y` into `pose.position` (z = 0).
3. Calls `_yaw_to_quat` and stores the result in `pose.orientation`.

---

## 5. The Node Class: `MissionExecutorNode`

### 5.1 Construction (`__init__`)

The constructor follows the **declare → read → setup** pattern:

1. **Declare ROS parameters** with sensible defaults so the node runs out of the box:
   - `mission_sequence` — list of waypoint names to visit (default: `['A1', 'B2', 'DOCK']`)
   - `auto_return_to_dock` — whether to append a DOCK step automatically (default: `True`)
   - `map_frame` — TF reference frame for goal poses (default: `'map'`)

2. **Read the parameter values** into private instance attributes.

3. **Create the status publisher** on `/amr/mission_status` (type `std_msgs/String`).

4. **Instantiate `BasicNavigator`** — this is the Nav2 Python client that wraps all the Nav2 action servers.

5. **Initialise an empty report list** (`_reports`) that will grow as steps complete.

### 5.2 `run_sequence()`

The public entry point, called exactly once from `main()`:

1. Calls `waitUntilNav2Active()` to block until the Nav2 lifecycle nodes are ready.
2. Loops through every name in `_sequence`, calling `_go(name)` for each one.
3. After each step, appends the report and publishes status. If a step FAILED, the loop breaks immediately.
4. If `auto_return_to_dock` is `True` and the last step was not already DOCK, navigates to DOCK.
5. Calls `_print_summary()` to log the final results.

### 5.3 `_go(name)` — The Navigation Worker

This is the core logic of the file:

```
_go("A1")
  │
  ├─ validate name against SHELF_REGISTRY
  ├─ build PoseStamped with _make_pose()
  ├─ stamp header with current sim time
  ├─ call nav.goToPose(pose)          ← non-blocking; Nav2 starts planning
  │
  └─ polling loop (every 100 ms):
       ├─ check timeout (120 s) → cancelTask() + return CANCELLED
       ├─ every 5 s: log distance_remaining and ETA from getFeedback()
       └─ when isTaskComplete() → read getResult():
            SUCCEEDED → return MissionReport(..., SUCCEEDED)
            CANCELED  → return MissionReport(..., CANCELLED)
            other     → return MissionReport(..., FAILED)
```

The 100 ms sleep inside the loop keeps CPU usage low while still being responsive.

### 5.4 `_pub_status(report)`

Formats a colon-separated status string and publishes it:

```
SUCCEEDED:A1:4.3s
FAILED:B2:7.1s:Nav2 failed
CANCELLED:C1:120.0s:Timeout
```

### 5.5 `_print_summary()`

After the mission loop, logs a table with a ✔/✘ icon, waypoint name, status, elapsed time, and optional detail for each step, then prints the total success count and mission duration.

---

## 6. Entry Point (`main`)

```python
rclpy.init(args=args)          # start ROS 2
node = MissionExecutorNode()   # build the node
node.run_sequence()            # execute mission (blocks)
node.destroy_node()            # clean up
rclpy.shutdown()               # stop ROS 2
```

A `try/except KeyboardInterrupt` block catches Ctrl-C: it cancels any in-flight Nav2 task before shutting down, preventing the robot from continuing to drive to a goal nobody is monitoring.

---

## 7. Configuration Examples

**Default sequence (A1 → B2 → DOCK):**
```bash
ros2 run warehouse_amr mission_executor.py
```

**Custom sequence via CLI:**
```bash
ros2 run warehouse_amr mission_executor.py --ros-args \
    -p mission_sequence:='["A3","C1","DOCK"]'
```

**Disable auto-dock:**
```bash
ros2 run warehouse_amr mission_executor.py --ros-args \
    -p auto_return_to_dock:=false
```

**Via launch file (`mission.launch.py`):**
```bash
ros2 launch warehouse_amr mission.launch.py
```
The launch file starts the full simulation first, waits 30 seconds for Nav2 to initialise, then starts the executor with the sequence `["A1","B2","C3","DOCK"]`.

---

## 8. Class / Function Dependency Graph

```
main()
 └─► MissionExecutorNode()
       ├─► __init__()
       │     ├─ declare_parameter() × 3
       │     ├─ create_publisher()
       │     └─ BasicNavigator()
       └─► run_sequence()
             ├─ waitUntilNav2Active()
             ├─► _go(name)            [per step]
             │     ├─► _make_pose()
             │     │     └─► _yaw_to_quat()
             │     ├─ goToPose()
             │     └─ polling loop
             ├─► _pub_status(report)  [per step]
             └─► _print_summary()     [once, at end]
```

---

## 9. Key Design Decisions

| Decision | Reason |
|----------|--------|
| `BasicNavigator` instead of a raw action client | Hides Nav2 lifecycle management and action boilerplate; simpler code |
| Polling loop with `time.sleep(0.1)` | Keeps the thread responsive without busy-spinning; avoids needing a separate ROS executor thread |
| `auto_return_to_dock` parameter | Allows fleet managers to disable auto-dock when the robot needs to stay at the last pick location |
| 120 s timeout | Long enough for complex paths in a large warehouse; prevents the robot from hanging indefinitely if Nav2 gets stuck |
| Status string format `STATUS:WAYPOINT:TIMEs` | Simple, grep-friendly; easily parsed by downstream monitoring tools |
