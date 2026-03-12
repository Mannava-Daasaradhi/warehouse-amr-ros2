"""
mission.launch.py
─────────────────
Full autonomous mission launch file.
Includes the full simulation (Gazebo + SLAM + Nav2 + RViz2) via sim.launch.py,
then starts the mission executor node with a 30-second delay so Nav2 has
time to fully initialise before the first navigation goal is sent.

Mirrors professor's walk.launch.py pattern:
  IncludeLaunchDescription(sim.launch.py) + TimerAction(30s, mission_executor).

Usage
~~~~~
  ros2 launch warehouse_amr mission.launch.py

To customise the waypoint sequence, edit the 'mission_sequence' parameter below,
or run the executor separately with --ros-args after the simulation is up:
  ros2 run warehouse_amr mission_executor.py --ros-args \
      -p mission_sequence:='["B1","C2","A3","DOCK"]'
"""

import os                                                              # os.path.join for cross-platform paths
from ament_index_python.packages import get_package_share_directory   # resolves installed package share paths
from launch import LaunchDescription                                   # container for all launch actions
from launch.actions import IncludeLaunchDescription, TimerAction       # include another launch file; delay actions
from launch.launch_description_sources import PythonLaunchDescriptionSource  # source type for Python launch files
from launch_ros.actions import Node                                    # declares a single ROS 2 node


PKG = 'warehouse_amr'  # ROS 2 package name used to locate installed share files


def generate_launch_description():
    pkg_dir = get_package_share_directory(PKG)  # absolute path to share/warehouse_amr/ in the install tree

    # ── Step 1: Include the full simulation ───────────────────────────────────
    # sim.launch.py starts everything needed to simulate the robot:
    #   Gazebo + URDF + bridge + EKF + SLAM Toolbox + Nav2 + RViz2
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'sim.launch.py')  # path to the simulation launch file
        ),
    )

    # ── Step 2: Mission executor node — delayed 30 s ──────────────────────────
    # The full Nav2 stack (planner, controller, bt_navigator) takes ~15–25 seconds
    # to come up after Gazebo starts.  A 30-second delay ensures Nav2 is fully
    # active before the first goToPose() goal is sent.
    # The executor also calls waitUntilNav2Active() as an extra safety check.
    mission_node = TimerAction(
        period=30.0,        # wait 30 seconds after launch before starting the executor
        actions=[
            Node(
                package='warehouse_amr',
                executable='mission_executor.py',       # installed by CMakeLists.txt install(PROGRAMS ...)
                name='warehouse_mission_executor',       # ROS 2 node name (visible in 'ros2 node list')
                output='screen',                        # print all log output to the terminal
                parameters=[{
                    # Ordered list of waypoint names to visit.
                    # Valid names: A1 A2 A3 B1 B2 B3 C1 C2 C3 STAGING DOCK
                    'mission_sequence': ['A1', 'B2', 'C3', 'DOCK'],

                    # If True and the last step is not already DOCK, automatically
                    # append a DOCK step so the robot always returns to charge.
                    'auto_return_to_dock': True,

                    # TF reference frame for all navigation goal poses.
                    # Must match the global_frame in nav2_params.yaml (bt_navigator section).
                    'map_frame': 'map',
                }],
            ),
        ],
    )

    return LaunchDescription([
        sim_launch,     # start Gazebo + SLAM + Nav2 + RViz2 immediately
        mission_node,   # start the autonomous mission executor after a 30-second delay
    ])
