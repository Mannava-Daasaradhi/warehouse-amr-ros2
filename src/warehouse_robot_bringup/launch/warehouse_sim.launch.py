import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

PKG_DESCRIPTION = "warehouse_robot_description"
PKG_GAZEBO      = "warehouse_robot_gazebo"
PKG_NAVIGATION  = "warehouse_robot_navigation"
PKG_BRINGUP     = "warehouse_robot_bringup"

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam         = LaunchConfiguration("slam")
    map_yaml     = LaunchConfiguration("map")
    run_rviz     = LaunchConfiguration("rviz")
    log_level    = LaunchConfiguration("log_level")

    # Absolute path resolved at launch time — passed into xacro as an arg
    controllers_yaml = os.path.join(
        get_package_share_directory(PKG_DESCRIPTION),
        "config", "ros2_controllers.yaml"
    )

    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution(
            [FindPackageShare(PKG_NAVIGATION), "maps", "warehouse_map.yaml"])),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    # Pass controllers_yaml as xacro arg so it embeds as absolute path in URDF
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare(PKG_DESCRIPTION), "urdf", "warehouse_robot.urdf.xacro"]),
        " controllers_yaml:=", controllers_yaml,
    ])
    robot_description = {"robot_description": robot_description_content}

    # ── Gazebo ──
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r -v2 ", PathJoinSubstitution(
                [FindPackageShare(PKG_GAZEBO), "worlds", "warehouse.sdf"])],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Robot state publisher ──
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ── Spawn robot ──
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_warehouse_amr",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name",  "warehouse_amr",
            "-x", "0.5", "-y", "0.5", "-z", "0.01", "-Y", "-1.5707963",
        ],
    )

    # ── GZ-ROS bridge ──
    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/diff_drive_controller/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
    )

    # ── ros2_control spawners ──
    # 15s delay: Gazebo needs time to load robot, init plugin, start controller_manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    controllers_delayed = TimerAction(
        period=15.0,
        actions=[joint_state_broadcaster_spawner, diff_drive_controller_spawner],
    )

    # ── EKF ──
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare(PKG_NAVIGATION), "config", "ekf.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── SLAM Toolbox ──
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "slam_params_file": PathJoinSubstitution(
                [FindPackageShare(PKG_NAVIGATION), "config", "nav2_params.yaml"]),
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(slam),
    )

    # ── Map server + AMCL (localization mode only) ──
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml}, {"use_sim_time": use_sim_time}],
        condition=UnlessCondition(slam),
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare(PKG_NAVIGATION), "config", "nav2_params.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        condition=UnlessCondition(slam),
    )
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                     "node_names": ["map_server", "amcl"]}],
        condition=UnlessCondition(slam),
    )

    # ── Custom Nav2 (no docking server) ──
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(PKG_BRINGUP), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": PathJoinSubstitution(
                [FindPackageShare(PKG_NAVIGATION), "config", "nav2_params.yaml"]),
        }.items(),
    )

    # ── RViz2 ──
    rviz2 = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(run_rviz),
    )

    # Spawn robot after RSP starts publishing robot_description
    spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawn_entity],
        )
    )

    # Nav2 delayed 8s — lets SLAM initialize first
    nav2_delayed = TimerAction(period=8.0, actions=[nav2])

    return LaunchDescription([
        *declared_args,
        gazebo,
        robot_state_publisher,
        spawn_after_rsp,
        gz_ros_bridge,
        controllers_delayed,
        robot_localization,
        slam_toolbox,
        map_server, amcl, lifecycle_manager_localization,
        nav2_delayed,
        rviz2,
    ])
