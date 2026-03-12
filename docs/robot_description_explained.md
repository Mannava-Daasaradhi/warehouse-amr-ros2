# Robot Description — `warehouse_robot.urdf.xacro` Explained

> **File:** `src/warehouse_amr/urdf/warehouse_robot.urdf.xacro`  
> **Format:** URDF + xacro macros + embedded Gazebo plugins  
> **Used by:** `sim.launch.py` (processed at launch time via `xacro.process_file()`)

---

## 1. What Is a URDF and Why Xacro?

**URDF (Unified Robot Description Format)** is an XML file that describes a robot's
physical structure to ROS 2.  It lists every rigid body (called a **link**) and
every connection between them (called a **joint**).

**Xacro** is a macro preprocessor for URDF.  Instead of repeating the same XML block
for the left wheel and the right wheel, xacro lets you write a `<xacro:macro>` once
and call it twice with different arguments.  The result is cleaner, less error-prone,
and easier to tune.

At launch time, `sim.launch.py` calls:
```python
robot_description = xacro.process_file(xacro_file, mappings={...}).toxml()
```
This expands all macros and substitutions into a plain URDF string, which is then
published on the `/robot_description` topic.

---

## 2. Physical Robot Specifications

These properties are defined as xacro variables at the top of the file and are reused
throughout every macro that references them.

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `base_length` | 0.60 m | Robot chassis front-to-back length |
| `base_width` | 0.50 m | Robot chassis left-to-right width |
| `base_height` | 0.25 m | Robot chassis height |
| `base_mass` | 25.0 kg | Total chassis mass (affects physics simulation) |
| `wheel_radius` | 0.10 m | Drive wheel radius (determines ground clearance and odometry scale) |
| `wheel_width` | 0.05 m | Drive wheel thickness |
| `wheel_mass` | 1.5 kg | Mass of each drive wheel |
| `wheel_sep` | 0.52 m | Distance between the two drive wheel centres (used by diff_drive_controller) |
| `caster_radius` | 0.04 m | Radius of passive caster balls |
| `caster_mass` | 0.5 kg | Mass of each caster |
| `lidar_mass` | 0.3 kg | Mass of the LiDAR unit (affects inertia tensor) |
| `imu_mass` | 0.05 kg | Mass of the IMU unit |

> **Important:** `wheel_sep` and `wheel_radius` must match the values in
> `config/ros2_controllers.yaml` exactly.  If they differ, odometry will be wrong.

---

## 3. Inertia Macros

Correct inertia tensors are required for realistic physics in Gazebo.  The xacro file
defines three macros that compute inertia from mass and shape:

### `box_inertia(m, x, y, z)`
Used for the chassis (`base_link`).  A uniform solid box has:
```
Ixx = (1/12) * m * (y² + z²)
Iyy = (1/12) * m * (x² + z²)
Izz = (1/12) * m * (x² + y²)
```

### `cylinder_inertia(m, r, h)`
Used for the drive wheels.  A solid cylinder rotating about its symmetry axis:
```
Ixx = Iyy = (1/12) * m * (3r² + h²)
Izz = (1/2) * m * r²
```

### `sphere_inertia(m, r)`
Used for the passive casters:
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

All off-diagonal terms (`ixy`, `ixz`, `iyz`) are zero because the shapes are symmetric.

---

## 4. Link and Joint Hierarchy

The robot's coordinate frames form a tree rooted at `base_footprint`.

```
base_footprint               ← 2-D ground projection; used by Nav2 as the planning frame
 └── base_link               ← Centre of chassis, raised by wheel_radius (0.10 m) above ground
      ├── left_wheel_link    ← Left drive wheel, offset +0.26 m in Y
      ├── right_wheel_link   ← Right drive wheel, offset −0.26 m in Y
      ├── front_caster_link  ← Front passive caster (fixed joint — no motor)
      ├── rear_caster_link   ← Rear passive caster (fixed joint — no motor)
      ├── lidar_link         ← LiDAR sensor origin (fixed joint)
      └── imu_link           ← IMU sensor origin (fixed joint)
```

### Joint Types

| Joint | Type | Why |
|-------|------|-----|
| `base_footprint_joint` | `fixed` | The footprint never moves relative to the chassis |
| `left_wheel_joint` | `continuous` | Wheels spin freely (no angle limit) |
| `right_wheel_joint` | `continuous` | Wheels spin freely |
| All caster joints | `fixed` | Casters are passive balls; the physics engine simulates their rolling |
| `lidar_joint` | `fixed` | Sensor doesn't move |
| `imu_joint` | `fixed` | Sensor doesn't move |

### Key Offsets

| Child Frame | Offset from Parent | Reason |
|-------------|-------------------|--------|
| `base_link` from `base_footprint` | z = +0.10 m (= `wheel_radius`) | The chassis sits on top of the wheels |
| `left_wheel_link` from `base_link` | y = +0.26 m (= `wheel_sep / 2`) | Symmetric left placement |
| `right_wheel_link` from `base_link` | y = −0.26 m (= `-wheel_sep / 2`) | Symmetric right placement |
| `front_caster_link` from `base_link` | x = +0.257 m, z = −0.06 m | Forward edge, dropped to ground level |
| `rear_caster_link` from `base_link` | x = −0.257 m, z = −0.06 m | Rear edge, dropped to ground level |
| `lidar_link` from `base_link` | x = +0.23 m, z = +0.285 m | Forward-top mounting point |
| `imu_link` from `base_link` | z = +0.125 m (= `base_height / 2`) | Geometric centre of chassis |

---

## 5. The Drive Wheel Macro

```xacro
<xacro:macro name="drive_wheel" params="prefix reflect">
```

- **`prefix`** — either `"left"` or `"right"`.  Used to name the link and joint.
- **`reflect`** — either `1` (left) or `-1` (right).  Multiplied by `wheel_sep/2` to
  place the wheel on the correct side.

The macro creates one `<link>` (visual + collision + inertia) and one `<joint>` that
connects the wheel to `base_link`.  It also sets the Gazebo friction coefficient
(`mu1 = mu2 = 0.8`) to give the wheels enough grip on the floor.

---

## 6. Caster Macro

```xacro
<xacro:macro name="caster" params="prefix x_pos">
```

Passive casters support the chassis but are not driven.  Their Gazebo friction is
set very low (`mu1 = mu2 = 0.01`) so they slide freely, simulating a ball caster.

The z-position is `−wheel_radius + caster_radius` — this places the caster sphere
so it just touches the floor (same ground plane as the drive wheels).

---

## 7. Sensor Configuration

### LiDAR (`lidar_link`)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `update_rate` | 10 Hz | Scans per second |
| `topic` | `/scan` | Publishes `sensor_msgs/LaserScan` here |
| `samples` | 720 | Rays in a full 360° sweep (0.5° angular resolution) |
| `min_angle` | −π rad | Start of sweep (−180°) |
| `max_angle` | +π rad | End of sweep (+180°) |
| `min` range | 0.10 m | Anything closer is ignored |
| `max` range | 12.0 m | Maximum detection range |
| noise stddev | 0.01 m | Small Gaussian noise to simulate real sensor |

The LiDAR is mounted at the **front** of the chassis (`x = base_length/2 − 0.07`)
and **above** the chassis top (`z = base_height + 0.035`), giving it a clear 360° view.

### IMU (`imu_link`)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `update_rate` | 100 Hz | Measurements per second |
| `topic` | `/imu/data_raw` | Publishes `sensor_msgs/Imu` here |
| angular velocity noise stddev | 0.009 rad/s | Gyroscope noise level |
| linear acceleration noise stddev | 0.021 m/s² | Accelerometer noise level |

The IMU is at the **geometric centre** of the chassis (`z = base_height / 2 = 0.125`).
The EKF fuses IMU yaw-rate with wheel odometry to produce a better pose estimate.

---

## 8. ros2_control Integration

The URDF contains a `<ros2_control>` block that declares which joints are controlled
by `ros2_control` and what interfaces they expose:

```xml
<ros2_control name="warehouse_amr_hardware" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>   <!-- Gazebo as the hardware backend -->
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>   <!-- diff_drive_controller sends velocity commands here -->
    <state_interface name="position"/>     <!-- encoder position read back -->
    <state_interface name="velocity"/>     <!-- encoder velocity read back -->
  </joint>
  <!-- right_wheel_joint is identical -->
</ros2_control>
```

The plugin `gz_ros2_control/GazeboSimSystem` makes Gazebo's physics act as the
hardware backend: instead of sending signals to real motors, the `controller_manager`
drives Gazebo joint positions/velocities directly.

---

## 9. Gazebo Plugins

### gz_ros2_control Plugin

```xml
<plugin filename="gz_ros2_control-system"
        name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>$(arg controllers_yaml)</parameters>
```

This plugin starts the `controller_manager` inside Gazebo and loads the
`diff_drive_controller` and `joint_state_broadcaster` whose parameters are
defined in `config/ros2_controllers.yaml`.

The path to `ros2_controllers.yaml` is passed in at launch time via the xacro arg
`controllers_yaml`.  This is why `sim.launch.py` passes:
```python
mappings={'controllers_yaml': controllers_yaml}
```

### Joint State Publisher Plugin

```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <topic>joint_states</topic>
</plugin>
```

Publishes `/joint_states` (`sensor_msgs/JointState`) so `robot_state_publisher`
can compute the dynamic TF transforms for the spinning wheels.

---

## 10. Gazebo Visual Materials

Each link gets a Gazebo material colour via a `<gazebo reference="...">` block:

| Link | Colour | Why |
|------|--------|-----|
| `base_link` | Orange | Distinctive warehouse robot colour |
| Drive wheels | Dark Grey | Rubber-like appearance |
| Casters | Grey | Generic metal/plastic casters |
| `lidar_link` | Black | Typical LiDAR housing colour |

---

## 11. How to Modify the Robot

### Change the robot size

Edit the `<xacro:property>` values at the top of the file, then rebuild:
```bash
colcon build --packages-select warehouse_amr --symlink-install
```
Any value used in a formula (e.g. `${base_length/2 - 0.07}`) will update automatically.

### Change wheel separation

Update **both** the `wheel_sep` property in the xacro file **and** the
`wheel_separation` value in `config/ros2_controllers.yaml`.  They must match or
odometry will be off.

### Add a sensor

1. Add a `<link>` with visual/collision/inertial elements.
2. Add a `<joint type="fixed">` connecting it to `base_link`.
3. Add a `<gazebo reference="..."><sensor ...>` block with the plugin config.
4. Add the bridging entry in `sim.launch.py`'s `bridge_args` list.

---

## 12. Quick Reference

```
robot name  : warehouse_amr
base frame  : base_footprint  (used by Nav2 and EKF)
chassis     : 0.60 × 0.50 × 0.25 m  |  25 kg
drive       : differential (2 × driven + 2 × caster)
wheel sep   : 0.52 m  |  wheel radius: 0.10 m
lidar       : 360°, 12 m range, 10 Hz, 720 rays, topic=/scan
imu         : 100 Hz, topic=/imu/data_raw
controller  : diff_drive_controller (ros2_control)
simulator   : Gazebo Harmonic (gz_ros2_control plugin)
```
