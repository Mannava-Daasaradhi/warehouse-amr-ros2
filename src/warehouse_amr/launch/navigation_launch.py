"""
navigation_launch.py
────────────────────
Launches all Nav2 servers plus a lifecycle manager that brings them up in order.
Mirrors the professor's pattern of one Node() per server, all explicit.

Included by sim.launch.py with an 8-second delay so SLAM Toolbox has time
to publish the first /map message before Nav2 tries to initialise.

Args
~~~~
  use_sim_time  (default: true)   — whether Nav2 uses /clock (simulation time)
  params_file   (required)        — absolute path to nav2_params.yaml
"""

from launch import LaunchDescription                   # container for all launch actions
from launch.actions import DeclareLaunchArgument       # declares a user-configurable CLI argument
from launch.substitutions import LaunchConfiguration   # lazy reference to a declared argument value
from launch_ros.actions import Node                    # declares a single ROS 2 node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')  # reference to the use_sim_time argument
    params_file  = LaunchConfiguration('params_file')   # reference to the nav2_params.yaml path

    return LaunchDescription([
        # ── Declare arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='true'),  # default: simulation time
        DeclareLaunchArgument('params_file'),                          # no default — must be provided by caller

        # ── Nav2 server nodes (one per server — professor's explicit style) ──
        # Each server is an independent ROS 2 node.  All share the same params_file
        # and use_sim_time setting.  The lifecycle_manager (below) activates them
        # in the correct dependency order.
        Node(package='nav2_controller',        executable='controller_server',   name='controller_server',   output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # local planner: global path → /cmd_vel_smoothed
        Node(package='nav2_smoother',          executable='smoother_server',     name='smoother_server',     output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # post-processes global path to remove sharp corners
        Node(package='nav2_planner',           executable='planner_server',      name='planner_server',      output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # global planner (Dijkstra/A*): current pose → goal → /plan
        Node(package='nav2_behaviors',         executable='behavior_server',     name='behavior_server',     output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # recovery behaviours: spin, backup, wait, drive_on_heading
        Node(package='nav2_bt_navigator',      executable='bt_navigator',        name='bt_navigator',        output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # top-level action server: goToPose() → behaviour tree
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',   name='waypoint_follower',   output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # drives through a list of waypoints in sequence
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',   name='velocity_smoother',   output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # enforces acceleration limits on velocity commands
        Node(package='nav2_collision_monitor', executable='collision_monitor',   name='collision_monitor',   output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),   # safety gate: stops robot if obstacle about to be hit

        # ── Lifecycle Manager ─────────────────────────────────────────────────
        # Manages all Nav2 nodes through the ROS 2 lifecycle state machine:
        #   unconfigured → inactive → active
        # autostart=True means it does this automatically at startup so the
        # operator does not need to manually trigger lifecycle transitions.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,                  # automatically configure and activate all managed nodes
                'node_names': [                     # managed in this order (order matters for dependencies)
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }],
        ),
    ])
