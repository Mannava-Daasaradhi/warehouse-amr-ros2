"""
mission.launch.py
─────────────────
Includes the full sim launch then starts the mission executor after a delay.
Mirrors professor's walk.launch.py pattern:
  IncludeLaunchDescription(gazebo.launch.py) + TimerAction(walk_node).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


PKG = 'warehouse_amr'


def generate_launch_description():
    pkg_dir = get_package_share_directory(PKG)

    # Include the full simulation (Gazebo + SLAM + Nav2 + RViz)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'sim.launch.py')
        ),
    )

    # Mission executor node — delayed 30s to let Nav2 fully initialize
    mission_node = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='warehouse_amr',
                executable='mission_executor.py',
                name='warehouse_mission_executor',
                output='screen',
                parameters=[{
                    'mission_sequence': ['A1', 'B2', 'C3', 'DOCK'],
                    'auto_return_to_dock': True,
                    'map_frame': 'map',
                }],
            ),
        ],
    )

    return LaunchDescription([
        sim_launch,
        mission_node,
    ])
