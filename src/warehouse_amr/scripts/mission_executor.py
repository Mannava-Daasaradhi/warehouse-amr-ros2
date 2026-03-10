#!/usr/bin/env python3
"""
mission_executor.py
───────────────────
Warehouse AMR mission executor — Nav2 BasicNavigator client.

Follows the professor's walk.py style:
  · declare_parameter() for every tunable value
  · clean __init__ / callback structure
  · standalone main() with spin/shutdown pattern

Usage
~~~~~
  ros2 run warehouse_amr mission_executor.py
  ros2 run warehouse_amr mission_executor.py --ros-args \
      -p mission_sequence:='["A1","B3","DOCK"]'
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String


# ─────────────────────────────────────────────────────────────────────────────
# Data types
# ─────────────────────────────────────────────────────────────────────────────

class MissionStatus(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    SUCCEEDED  = auto()
    FAILED     = auto()
    CANCELLED  = auto()


@dataclass
class Waypoint:
    """Named pose in the map frame."""
    name: str
    x: float
    y: float
    yaw: float          # radians


@dataclass
class MissionReport:
    waypoint: str
    status: MissionStatus
    elapsed_sec: float
    message: str = ""


# ─────────────────────────────────────────────────────────────────────────────
# Shelf / dock registry — edit x/y to match your map
# ─────────────────────────────────────────────────────────────────────────────

SHELF_REGISTRY: dict[str, Waypoint] = {
    "A1": Waypoint("A1", x=2.0, y=3.5, yaw=0.0),
    "A2": Waypoint("A2", x=2.0, y=5.0, yaw=0.0),
    "A3": Waypoint("A3", x=2.0, y=6.5, yaw=0.0),
    "B1": Waypoint("B1", x=5.0, y=3.5, yaw=math.pi),
    "B2": Waypoint("B2", x=5.0, y=5.0, yaw=math.pi),
    "B3": Waypoint("B3", x=5.0, y=6.5, yaw=math.pi),
    "C1": Waypoint("C1", x=8.0, y=3.5, yaw=0.0),
    "C2": Waypoint("C2", x=8.0, y=5.0, yaw=0.0),
    "C3": Waypoint("C3", x=8.0, y=6.5, yaw=0.0),
    "STAGING": Waypoint("STAGING", x=1.0, y=1.0, yaw=0.0),
    "DOCK":    Waypoint("DOCK",    x=0.5, y=0.5, yaw=-math.pi / 2),
}


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _make_pose(wp: Waypoint, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = wp.x
    pose.pose.position.y = wp.y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = _yaw_to_quat(wp.yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class MissionExecutorNode(Node):
    """
    Drives the AMR through a configurable waypoint sequence
    using Nav2 BasicNavigator.
    Follows professor's Node pattern: declare params → init pubs → create timer.
    """

    # Tuning constants — could also be ROS params
    NAV_TIMEOUT_S      = 120.0
    FEEDBACK_INTERVAL  = 5.0

    def __init__(self) -> None:
        super().__init__('warehouse_mission_executor')

        # ── Parameters (professor's declare_parameter style) ──────────────
        self.declare_parameter('mission_sequence',   value=['A1', 'B2', 'DOCK'])
        self.declare_parameter('auto_return_to_dock', True)
        self.declare_parameter('map_frame',          'map')

        self._sequence  = self.get_parameter('mission_sequence').get_parameter_value().string_array_value
        self._auto_dock = self.get_parameter('auto_return_to_dock').get_parameter_value().bool_value
        self._frame     = self.get_parameter('map_frame').get_parameter_value().string_value

        # ── Publisher (professor's create_publisher pattern) ───────────────
        self._status_pub = self.create_publisher(String, '/amr/mission_status', 10)

        # ── Nav2 navigator ─────────────────────────────────────────────────
        self._nav = BasicNavigator()

        self._reports: list[MissionReport] = []

        self.get_logger().info(f'Mission executor ready — sequence: {self._sequence}')

    # ── Public API ────────────────────────────────────────────────────────

    def run_sequence(self) -> None:
        """Execute the full mission sequence — called once from main()."""
        self.get_logger().info('─── Mission sequence starting ───')
        self._nav.waitUntilNav2Active()

        for step in self._sequence:
            report = self._go(step)
            self._reports.append(report)
            self._pub_status(report)
            if report.status == MissionStatus.FAILED:
                self.get_logger().error(f"Step '{step}' FAILED — aborting.")
                break

        # Auto-dock if last waypoint wasn't already DOCK
        if self._auto_dock and self._sequence[-1].upper() != 'DOCK':
            report = self._go('DOCK')
            self._reports.append(report)
            self._pub_status(report)

        self._print_summary()

    # ── Navigation ────────────────────────────────────────────────────────

    def _go(self, name: str) -> MissionReport:
        """Navigate to a named waypoint. Mirrors professor's publish_cmd simplicity."""
        key = name.upper()
        if key not in SHELF_REGISTRY:
            msg = f"Unknown waypoint '{key}'"
            self.get_logger().error(msg)
            return MissionReport(key, MissionStatus.FAILED, 0.0, msg)

        wp = SHELF_REGISTRY[key]
        self.get_logger().info(
            f'▶ → {wp.name}  ({wp.x:.2f}, {wp.y:.2f}, '
            f'yaw={math.degrees(wp.yaw):.1f}°)'
        )

        pose = _make_pose(wp, self._frame)
        pose.header.stamp = self._nav.get_clock().now().to_msg()
        self._nav.goToPose(pose)

        start    = time.monotonic()
        last_log = start

        while not self._nav.isTaskComplete():
            elapsed = time.monotonic() - start

            if elapsed > self.NAV_TIMEOUT_S:
                self.get_logger().warn(f"Timeout navigating to '{wp.name}'. Cancelling.")
                self._nav.cancelTask()
                return MissionReport(wp.name, MissionStatus.CANCELLED, elapsed, 'Timeout')

            if time.monotonic() - last_log >= self.FEEDBACK_INTERVAL:
                fb = self._nav.getFeedback()
                if fb:
                    eta  = Duration.from_msg(fb.estimated_time_remaining).nanoseconds / 1e9
                    dist = fb.distance_remaining
                    self.get_logger().info(f'  {wp.name}: {dist:.2f}m left, ETA {eta:.1f}s')
                last_log = time.monotonic()

            time.sleep(0.1)

        elapsed = time.monotonic() - start
        result  = self._nav.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'✔ {wp.name} reached in {elapsed:.1f}s')
            return MissionReport(wp.name, MissionStatus.SUCCEEDED, elapsed)
        elif result == TaskResult.CANCELED:
            return MissionReport(wp.name, MissionStatus.CANCELLED, elapsed, 'Cancelled')
        else:
            self.get_logger().error(f'✘ {wp.name} FAILED after {elapsed:.1f}s')
            return MissionReport(wp.name, MissionStatus.FAILED, elapsed, 'Nav2 failed')

    # ── Publishers ────────────────────────────────────────────────────────

    def _pub_status(self, r: MissionReport) -> None:
        msg = String()
        msg.data = f'{r.status.name}:{r.waypoint}:{r.elapsed_sec:.1f}s'
        if r.message:
            msg.data += f':{r.message}'
        self._status_pub.publish(msg)

    # ── Summary ───────────────────────────────────────────────────────────

    def _print_summary(self) -> None:
        self.get_logger().info('─── Mission Summary ───')
        total = 0.0
        for r in self._reports:
            icon = '✔' if r.status == MissionStatus.SUCCEEDED else '✘'
            self.get_logger().info(
                f'  {icon} {r.waypoint:<10} {r.status.name:<12} {r.elapsed_sec:6.1f}s'
                + (f'  ({r.message})' if r.message else '')
            )
            total += r.elapsed_sec
        ok = sum(1 for r in self._reports if r.status == MissionStatus.SUCCEEDED)
        self.get_logger().info(f'  {ok}/{len(self._reports)} succeeded | {total:.1f}s total')


# ─────────────────────────────────────────────────────────────────────────────
# Entry point — professor's main() pattern
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — cancelling task.')
        node._nav.cancelTask()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
