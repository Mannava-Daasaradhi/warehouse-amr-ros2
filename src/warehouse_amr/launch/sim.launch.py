"""
sim.launch.py
─────────────
Warehouse AMR simulation launch file.
Starts: Gazebo (warehouse world) + Robot State Publisher + Gazebo-ROS bridge +
        twist_stamper + ros2_control controllers + EKF + SLAM or AMCL + Nav2 + RViz2.

Follows the professor's br2_robodog pattern:
  - Direct xacro.process_file() for robot description
  - Explicit bridge_args list for every bridged topic
  - All nodes declared in one LaunchDescription
  - Conditional SLAM vs map-server localization

Args
~~~~
  slam    (default: true)  — run SLAM Toolbox mapping; false = AMCL localization
  rviz    (default: true)  — open RViz2
"""

import os
import xacro                                              # ROS xacro macro preprocessor for URDF files
from ament_index_python.packages import get_package_share_directory  # resolves installed package paths
from launch import LaunchDescription                      # container for all launch actions
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,      # declare CLI args; include other launch files
    TimerAction, RegisterEventHandler                     # delay actions; react to process events
)
from launch.conditions import IfCondition, UnlessCondition  # start nodes conditionally based on arg values
from launch.event_handlers import OnProcessStart          # fire an action when a specific process starts
from launch.launch_description_sources import PythonLaunchDescriptionSource  # include Python launch files
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution   # lazy argument references
from launch_ros.actions import Node                       # declares a single ROS 2 node to launch



PKG = 'warehouse_amr'  # ROS 2 package name; used to locate installed share files


def generate_launch_description():

    pkg_dir = get_package_share_directory(PKG)  # absolute path to share/warehouse_amr/ after 'colcon build'

    slam     = LaunchConfiguration('slam')      # lazy reference to the 'slam' launch argument
    run_rviz = LaunchConfiguration('rviz')      # lazy reference to the 'rviz' launch argument

    # ── Process xacro → URDF string (professor's direct approach) ──────────
    # xacro.process_file() runs at Python import time (before nodes start) and
    # expands all macros into a plain URDF XML string.
    # We pass the absolute path to ros2_controllers.yaml as a xacro argument so
    # the embedded gz_ros2_control plugin can locate its config file.
    xacro_file        = os.path.join(pkg_dir, 'urdf', 'warehouse_robot.urdf.xacro')
    controllers_yaml  = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'controllers_yaml': controllers_yaml}  # passes the YAML path into the xacro arg
    ).toxml()                                             # serialise the processed XML to a string

    # ── World file ──────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')  # Gazebo SDF world (shelves, walls)

    # ── Declared args ───────────────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('slam',  default_value='true',  choices=['true', 'false']),  # SLAM vs AMCL
        DeclareLaunchArgument('rviz',  default_value='true',  choices=['true', 'false']),  # show RViz2
    ]

    # ── Robot State Publisher ───────────────────────────────────────────────
    # Publishes /robot_description (URDF string) and all static TF transforms from
    # the URDF's fixed joints.  Also subscribes to /joint_states to broadcast
    # dynamic TF transforms for the rotating wheels.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,  # the full URDF XML as a single string parameter
            'use_sim_time': True,                    # use /clock topic (Gazebo simulation time)
        }],
    )

    # ── Gazebo Sim ──────────────────────────────────────────────────────────
    # Includes the standard gz_sim.launch.py which starts the Gazebo server and GUI.
    # -r = run the simulation clock immediately (don't wait for user to press Play)
    # -v2 = verbosity level 2 (moderate logging)
    # on_exit_shutdown = kill all ROS 2 nodes if the Gazebo window is closed
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v2 {world_file}',  # start simulation running with the warehouse world
            'on_exit_shutdown': 'true',          # terminate all nodes when Gazebo exits
        }.items(),
    )

    # ── Spawn robot (fires after RSP starts — professor's event-handler pattern) ──
    # The robot can only be spawned after robot_state_publisher publishes
    # /robot_description.  RegisterEventHandler + OnProcessStart ensures ordering
    # without using an arbitrary timer.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',                  # Gazebo entity creation CLI tool
        name='spawn_warehouse_amr',
        output='screen',
        arguments=[
            '-name',  'warehouse_amr',        # model name inside Gazebo
            '-topic', '/robot_description',   # read URDF from this ROS topic
            '-x', '0.5', '-y', '0.5', '-z', '0.01',  # spawn position (metres above floor)
            '-Y', '-1.5707963',               # yaw = -90° (facing south, -Y direction)
        ],
    )

    spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,  # wait for RSP to start
            on_start=[spawn_entity],              # then spawn the robot in Gazebo
        )
    )

    # ── GZ ←→ ROS Bridge (professor's explicit bridge_args list pattern) ───
    # Bidirectionally bridges Gazebo internal topics to ROS 2 topics.
    # Syntax:  /ros_topic@ROS_Type[gz.Type   = Gazebo → ROS (sensor data)
    #          /ros_topic@ROS_Type]gz.Type   = ROS → Gazebo (commands)
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',                           # sim time: Gazebo → ROS
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',                      # LiDAR: Gazebo → ROS
        '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',                         # IMU: Gazebo → ROS
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',                         # velocity commands: ROS → Gazebo
        '/diff_drive_controller/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',    # wheel odom: Gazebo → ROS
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',                              # TF transforms: Gazebo → ROS
    ]

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',      # single node that handles all bridged topics at once
        name='gz_ros_bridge',
        arguments=bridge_args,              # the list of topic bridges defined above
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── ros2_control spawners (delayed 15s — Gazebo needs time to init) ────
    # The spawner activates controllers by calling a service on /controller_manager.
    # controller_manager only exists after Gazebo loads the robot model and the
    # gz_ros2_control plugin initialises (~10 s).  15 s provides a safe margin.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',               # activates the named controller
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',               # activates the diff_drive_controller
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    controllers_delayed = TimerAction(
        period=15.0,                        # wait 15 seconds before activating controllers
        actions=[joint_state_broadcaster, diff_drive_controller],
    )

    # ── EKF (robot_localization) ────────────────────────────────────────────
    # Fuses wheel odometry and IMU data into a smooth pose estimate.
    # Publishes /odometry/filtered and broadcasts odom → base_footprint TF.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf.yaml'),  # EKF configuration
            {'use_sim_time': True},                         # override to use sim clock
        ],
    )

    # ── SLAM Toolbox (mapping mode) ─────────────────────────────────────────
    # Builds a 2-D occupancy map from /scan and publishes /map + map→odom TF.
    # Only started when slam:=true (the default).
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py',
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(slam),        # only start if slam:=true
    )

    # ── Map Server + AMCL (localization mode — when slam:=false) ───────────
    # map_server loads the pre-built warehouse_map.pgm and publishes /map.
    # amcl localises the robot within that map using the LiDAR (particle filter).
    # lifecycle_manager_localization brings both nodes through configure → activate.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml')},  # pre-built map
            {'use_sim_time': True},
        ],
        condition=UnlessCondition(slam),    # only start if slam:=false
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),  # AMCL parameters
            {'use_sim_time': True},
        ],
        condition=UnlessCondition(slam),    # only start if slam:=false
    )
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,                          # configure and activate on startup
            'node_names': ['map_server', 'amcl'],       # nodes to manage
        }],
        condition=UnlessCondition(slam),    # only start if slam:=false
    )

    # ── Nav2 stack (delayed 8s — SLAM needs to come up first) ──────────────
    # navigation_launch.py starts all Nav2 servers and their lifecycle manager.
    # 8 seconds gives SLAM Toolbox time to publish a first /map message and
    # for the TF tree (map → odom → base_footprint) to be complete.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),  # Nav2 parameters
        }.items(),
    )
    nav2_delayed = TimerAction(period=8.0, actions=[nav2])  # wait 8 s before starting Nav2

    # ── Twist → TwistStamped (ros2_controllers 4.x on Jazzy requires TwistStamped) ──
    # Nav2 publishes geometry_msgs/Twist on /cmd_vel.
    # ros2_controllers 4.x (Jazzy) requires geometry_msgs/TwistStamped on the controller input.
    # twist_stamper adds a header (frame_id + timestamp) to convert between the two types.
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': True, 'frame_id': 'base_link'}],  # stamp with base_link frame
        remappings=[
            ('/cmd_vel_in',  '/cmd_vel'),                       # read plain Twist from Nav2/collision_monitor
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'), # write TwistStamped to the controller
        ],
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────
    # Opens the pre-configured RViz2 layout showing the map, robot model,
    # LiDAR scan, costmaps, and planned paths.
    # Only started when rviz:=true (the default).
    rviz_config = os.path.join(pkg_dir, 'rviz', 'warehouse_nav.rviz')  # pre-configured display layout
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],      # load the saved RViz2 configuration file
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(run_rviz),    # only start if rviz:=true
    )

    return LaunchDescription([
        *declared_args,             # slam and rviz launch arguments
        gz_sim,                     # Gazebo Sim (warehouse world)
        robot_state_publisher,      # URDF → /robot_description + static TF
        spawn_after_rsp,            # spawn robot in Gazebo after RSP starts
        gz_bridge,                  # Gazebo ↔ ROS 2 topic bridge
        twist_stamper,              # Twist → TwistStamped converter
        controllers_delayed,        # ros2_control controller activation (at 15 s)
        ekf_node,                   # wheel odom + IMU → /odometry/filtered
        slam_toolbox,               # SLAM mapping (if slam:=true)
        map_server, amcl, lifecycle_localization,  # localization mode (if slam:=false)
        nav2_delayed,               # full Nav2 stack (at 8 s)
        rviz2,                      # visualisation (if rviz:=true)
    ])
