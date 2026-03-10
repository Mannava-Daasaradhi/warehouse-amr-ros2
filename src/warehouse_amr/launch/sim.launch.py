"""
sim.launch.py
─────────────
Warehouse AMR simulation launch file.
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
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PKG = 'warehouse_amr'


def generate_launch_description():

    pkg_dir = get_package_share_directory(PKG)

    slam     = LaunchConfiguration('slam')
    run_rviz = LaunchConfiguration('rviz')

    # ── Process xacro → URDF string (professor's direct approach) ──────────
    xacro_file        = os.path.join(pkg_dir, 'urdf', 'warehouse_robot.urdf.xacro')
    controllers_yaml  = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'controllers_yaml': controllers_yaml}
    ).toxml()

    # ── World file ──────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')

    # ── Declared args ───────────────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('slam',  default_value='true',  choices=['true', 'false']),
        DeclareLaunchArgument('rviz',  default_value='true',  choices=['true', 'false']),
    ]

    # ── Robot State Publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── Gazebo Sim ──────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v2 {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Spawn robot (fires after RSP starts — professor's event-handler pattern) ──
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_warehouse_amr',
        output='screen',
        arguments=[
            '-name',  'warehouse_amr',
            '-topic', '/robot_description',
            '-x', '0.5', '-y', '0.5', '-z', '0.01', '-Y', '-1.5707963',
        ],
    )

    spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawn_entity],
        )
    )

    # ── GZ ←→ ROS Bridge (professor's explicit bridge_args list pattern) ───
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/diff_drive_controller/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        arguments=bridge_args,
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── ros2_control spawners (delayed 15s — Gazebo needs time to init) ────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    controllers_delayed = TimerAction(
        period=15.0,
        actions=[joint_state_broadcaster, diff_drive_controller],
    )

    # ── EKF (robot_localization) ────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
    )

    # ── SLAM Toolbox (mapping mode) ─────────────────────────────────────────
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
        condition=IfCondition(slam),
    )

    # ── Map Server + AMCL (localization mode — when slam:=false) ───────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml')},
            {'use_sim_time': True},
        ],
        condition=UnlessCondition(slam),
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            {'use_sim_time': True},
        ],
        condition=UnlessCondition(slam),
    )
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=UnlessCondition(slam),
    )

    # ── Nav2 stack (delayed 8s — SLAM needs to come up first) ──────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        }.items(),
    )
    nav2_delayed = TimerAction(period=8.0, actions=[nav2])

    # ── Twist → TwistStamped (ros2_controllers 4.x on Jazzy requires TwistStamped) ──
    # Converts /cmd_vel (Twist) → /diff_drive_controller/cmd_vel (TwistStamped)
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': True, 'frame_id': 'base_link'}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ],
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_dir, 'rviz', 'warehouse_nav.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(run_rviz),
    )

    return LaunchDescription([
        *declared_args,
        gz_sim,
        robot_state_publisher,
        spawn_after_rsp,
        gz_bridge,
        twist_stamper,
        controllers_delayed,
        ekf_node,
        slam_toolbox,
        map_server, amcl, lifecycle_localization,
        nav2_delayed,
        rviz2,
    ])
