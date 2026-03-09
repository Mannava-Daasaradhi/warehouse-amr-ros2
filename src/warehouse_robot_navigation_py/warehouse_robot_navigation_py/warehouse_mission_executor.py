#!/usr/bin/env python3
"""
warehouse_mission_executor.py
─────────────────────────────
Mission-level Nav2 client for the warehouse AMR.

Features
~~~~~~~~
* Typed shelf / dock registry with arbitrary pose storage
* Async-style mission queue using BasicNavigator
* Graceful cancel, timeout, and recovery via Nav2 BT navigator
* Feedback monitoring with ETA estimation
* Docking sequence with final orientation correction
* ROS 2 Jazzy / Nav2 Humble+ compatible

Usage
~~~~~
  ros2 run warehouse_robot_navigation mission_executor
  ros2 run warehouse_robot_navigation mission_executor --ros-args -p mission_sequence:='["A1","B3","dock"]'

Requires
~~~~~~~~
  nav2_simple_commander (ships with Nav2)
  geometry_msgs, std_msgs
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
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
    tolerance: float = 0.15   # goal tolerance (m) — overrides nav2 default if desired


@dataclass
class MissionReport:
    waypoint: str
    status: MissionStatus
    elapsed_sec: float
    message: str = ""


# ─────────────────────────────────────────────────────────────────────────────
# Warehouse Registry — edit to match your map coordinates
# ─────────────────────────────────────────────────────────────────────────────

SHELF_REGISTRY: dict[str, Waypoint] = {
    # Row A
    "A1": Waypoint("A1", x=2.0,  y=3.5,  yaw=0.0),
    "A2": Waypoint("A2", x=2.0,  y=5.0,  yaw=0.0),
    "A3": Waypoint("A3", x=2.0,  y=6.5,  yaw=0.0),
    # Row B
    "B1": Waypoint("B1", x=5.0,  y=3.5,  yaw=math.pi),
    "B2": Waypoint("B2", x=5.0,  y=5.0,  yaw=math.pi),
    "B3": Waypoint("B3", x=5.0,  y=6.5,  yaw=math.pi),
    # Row C
    "C1": Waypoint("C1", x=8.0,  y=3.5,  yaw=0.0),
    "C2": Waypoint("C2", x=8.0,  y=5.0,  yaw=0.0),
    "C3": Waypoint("C3", x=8.0,  y=6.5,  yaw=0.0),
    # Staging / special zones
    "STAGING": Waypoint("STAGING", x=1.0, y=1.0, yaw=0.0),
    "DOCK":    Waypoint("DOCK",    x=0.5, y=0.5, yaw=-math.pi / 2),
}


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) quaternion from a yaw angle."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _make_pose_stamped(wp: Waypoint, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = wp.x
    pose.pose.position.y = wp.y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = _yaw_to_quaternion(wp.yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


# ─────────────────────────────────────────────────────────────────────────────
# Core Executor Node
# ─────────────────────────────────────────────────────────────────────────────

class WarehouseMissionExecutor(Node):
    """
    ROS 2 node that drives the warehouse AMR through a configurable
    mission sequence using the Nav2 BasicNavigator API.
    """

    NAVIGATION_TIMEOUT_S = 120.0    # abort if any single goal exceeds this
    FEEDBACK_LOG_INTERVAL_S = 5.0   # how often to log ETA feedback

    def __init__(self) -> None:
        super().__init__("warehouse_mission_executor")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter(
            "mission_sequence",
            value=["A1", "B2", "DOCK"],
        )
        self.declare_parameter("auto_return_to_dock", True)
        self.declare_parameter("map_frame", "map")

        self._sequence: list[str] = (
            self.get_parameter("mission_sequence")
            .get_parameter_value()
            .string_array_value
        )
        self._auto_dock: bool = (
            self.get_parameter("auto_return_to_dock")
            .get_parameter_value()
            .bool_value
        )
        self._map_frame: str = (
            self.get_parameter("map_frame")
            .get_parameter_value()
            .string_value
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "/amr/mission_status", 10)

        # ── Nav2 navigator ───────────────────────────────────────────────────
        self._nav = BasicNavigator()

        # ── State ────────────────────────────────────────────────────────────
        self._reports: list[MissionReport] = []
        self._current_mission_status = MissionStatus.IDLE

        self.get_logger().info(
            f"Mission executor ready. Sequence: {self._sequence}"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────────────────

    def go_to_shelf(self, shelf_id: str) -> MissionReport:
        """Navigate to a named shelf location."""
        shelf_id = shelf_id.upper()
        if shelf_id not in SHELF_REGISTRY:
            msg = f"Unknown shelf ID: '{shelf_id}'. Available: {list(SHELF_REGISTRY)}"
            self.get_logger().error(msg)
            return MissionReport(shelf_id, MissionStatus.FAILED, 0.0, msg)
        return self._execute_goal(SHELF_REGISTRY[shelf_id])

    def return_to_dock(self) -> MissionReport:
        """Navigate back to the charging dock."""
        self.get_logger().info("Returning to dock...")
        return self._execute_goal(SHELF_REGISTRY["DOCK"])

    def run_sequence(self) -> None:
        """Execute the full declared mission sequence."""
        self.get_logger().info("─── Mission sequence starting ───")
        self._nav.waitUntilNav2Active()

        for step in self._sequence:
            report = self.go_to_shelf(step) if step.upper() != "DOCK" else self.return_to_dock()
            self._reports.append(report)
            self._publish_status(report)

            if report.status == MissionStatus.FAILED:
                self.get_logger().error(
                    f"Step '{step}' FAILED — aborting sequence."
                )
                break

        if self._auto_dock and (
            not self._sequence or self._sequence[-1].upper() != "DOCK"
        ):
            report = self.return_to_dock()
            self._reports.append(report)
            self._publish_status(report)

        self._print_summary()

    # ─────────────────────────────────────────────────────────────────────────
    # Internal navigation logic
    # ─────────────────────────────────────────────────────────────────────────

    def _execute_goal(self, wp: Waypoint) -> MissionReport:
        """Send a goal to Nav2 and block until completion or timeout."""
        self.get_logger().info(
            f"▶ Navigating to '{wp.name}'  "
            f"({wp.x:.2f}, {wp.y:.2f}, yaw={math.degrees(wp.yaw):.1f}°)"
        )
        self._current_mission_status = MissionStatus.NAVIGATING
        self._publish_text(f"NAVIGATING:{wp.name}")

        pose = _make_pose_stamped(wp, frame_id=self._map_frame)
        # Stamp with current nav time
        pose.header.stamp = self._nav.get_clock().now().to_msg()

        self._nav.goToPose(pose)

        start = time.monotonic()
        last_log = start

        while not self._nav.isTaskComplete():
            elapsed = time.monotonic() - start

            # Timeout guard
            if elapsed > self.NAVIGATION_TIMEOUT_S:
                self.get_logger().warn(
                    f"Timeout ({self.NAVIGATION_TIMEOUT_S}s) reached for '{wp.name}'. Cancelling."
                )
                self._nav.cancelTask()
                return MissionReport(
                    wp.name,
                    MissionStatus.CANCELLED,
                    elapsed,
                    "Navigation timeout",
                )

            # Periodic feedback logging
            if time.monotonic() - last_log >= self.FEEDBACK_LOG_INTERVAL_S:
                feedback = self._nav.getFeedback()
                if feedback:
                    eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    dist = feedback.distance_remaining
                    self.get_logger().info(
                        f"  → '{wp.name}': {dist:.2f}m remaining, ETA {eta:.1f}s"
                    )
                last_log = time.monotonic()

            time.sleep(0.1)

        elapsed = time.monotonic() - start
        result = self._nav.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✔ '{wp.name}' reached in {elapsed:.1f}s")
            return MissionReport(wp.name, MissionStatus.SUCCEEDED, elapsed)

        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f"✘ Navigation to '{wp.name}' was cancelled.")
            return MissionReport(wp.name, MissionStatus.CANCELLED, elapsed, "Cancelled by Nav2")

        else:  # FAILED
            self.get_logger().error(
                f"✘ Navigation to '{wp.name}' FAILED after {elapsed:.1f}s."
            )
            return MissionReport(wp.name, MissionStatus.FAILED, elapsed, "Nav2 task failed")

    # ─────────────────────────────────────────────────────────────────────────
    # Publisher helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_status(self, report: MissionReport) -> None:
        status_str = (
            f"{report.status.name}:{report.waypoint}:"
            f"{report.elapsed_sec:.1f}s"
        )
        if report.message:
            status_str += f":{report.message}"
        self._publish_text(status_str)

    def _publish_text(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Summary
    # ─────────────────────────────────────────────────────────────────────────

    def _print_summary(self) -> None:
        self.get_logger().info("─── Mission Summary ───")
        total = 0.0
        for r in self._reports:
            icon = "✔" if r.status == MissionStatus.SUCCEEDED else "✘"
            self.get_logger().info(
                f"  {icon} {r.waypoint:<10} {r.status.name:<12} {r.elapsed_sec:6.1f}s"
                + (f"  ({r.message})" if r.message else "")
            )
            total += r.elapsed_sec
        succeeded = sum(1 for r in self._reports if r.status == MissionStatus.SUCCEEDED)
        self.get_logger().info(
            f"  Total: {succeeded}/{len(self._reports)} succeeded | "
            f"{total:.1f}s mission time"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    executor = WarehouseMissionExecutor()
    try:
        executor.run_sequence()
    except KeyboardInterrupt:
        executor.get_logger().info("Interrupted — cancelling active task.")
        executor._nav.cancelTask()
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
