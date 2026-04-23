#!/usr/bin/env python3                  
# tells the OS to run this file with Python 3
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

from __future__ import annotations          # enables PEP 563 postponed evaluation of type hints

import math                                 # provides trigonometric functions (sin, cos, pi, degrees)
import time                                 # provides time.monotonic() for elapsed-time measurement
from dataclasses import dataclass           # @dataclass decorator auto-generates __init__, __repr__, etc.
from enum import Enum, auto                 # Enum base class + auto() for automatic integer values

import rclpy                                # core ROS 2 Python client library
from geometry_msgs.msg import PoseStamped   # ROS message: stamped 3-D pose (position + orientation)
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # high-level Nav2 API
from rclpy.duration import Duration         # ROS 2 duration type used to parse ETA from Nav2 feedback
from rclpy.node import Node                 # base class for every ROS 2 node
from std_msgs.msg import String             # ROS message: plain UTF-8 string, used for status updates


# ─────────────────────────────────────────────────────────────────────────────
# Data types
# ─────────────────────────────────────────────────────────────────────────────

class MissionStatus(Enum):                  # defines all possible states a navigation step can be in
    IDLE       = auto()                     # node is alive but no navigation has been requested yet
    NAVIGATING = auto()                     # a goToPose command is currently in flight
    SUCCEEDED  = auto()                     # Nav2 confirmed the robot reached the goal pose
    FAILED     = auto()                     # Nav2 reported an unrecoverable failure
    CANCELLED  = auto()                     # navigation was stopped (timeout or keyboard interrupt)


@dataclass                                  # auto-generates __init__, __repr__, and __eq__ for Waypoint
class Waypoint:
    """Named pose in the map frame."""
    name: str                               # human-readable label used as the registry key (e.g. "A1")
    x: float                                # target X position in the map frame (metres)
    y: float                                # target Y position in the map frame (metres)
    yaw: float                              # desired heading at arrival, in radians (0 = facing +X axis)


@dataclass                                  # auto-generates boilerplate for MissionReport
class MissionReport:
    waypoint: str                           # name of the waypoint this report covers
    status: MissionStatus                   # final outcome of the navigation step
    elapsed_sec: float                      # wall-clock seconds spent driving to this waypoint
    message: str = ""                       # optional human-readable detail (e.g. "Timeout", "Nav2 failed")


# ─────────────────────────────────────────────────────────────────────────────
# Shelf / dock registry — edit x/y to match your map
# ─────────────────────────────────────────────────────────────────────────────

SHELF_REGISTRY: dict[str, Waypoint] = {    # maps waypoint names to their map-frame poses
    "A1": Waypoint("A1", x=2.0, y=3.5, yaw=0.0),           # aisle A, bay 1 — robot faces +X
    "A2": Waypoint("A2", x=2.0, y=5.0, yaw=0.0),           # aisle A, bay 2 — robot faces +X
    "A3": Waypoint("A3", x=2.0, y=6.5, yaw=0.0),           # aisle A, bay 3 — robot faces +X
    "B1": Waypoint("B1", x=5.0, y=3.5, yaw=math.pi),       # aisle B, bay 1 — robot faces -X (180°)
    "B2": Waypoint("B2", x=5.0, y=5.0, yaw=math.pi),       # aisle B, bay 2 — robot faces -X (180°)
    "B3": Waypoint("B3", x=5.0, y=6.5, yaw=math.pi),       # aisle B, bay 3 — robot faces -X (180°)
    "C1": Waypoint("C1", x=8.0, y=3.5, yaw=0.0),           # aisle C, bay 1 — robot faces +X
    "C2": Waypoint("C2", x=8.0, y=5.0, yaw=0.0),           # aisle C, bay 2 — robot faces +X
    "C3": Waypoint("C3", x=8.0, y=6.5, yaw=0.0),           # aisle C, bay 3 — robot faces +X
    "STAGING": Waypoint("STAGING", x=1.0, y=1.0, yaw=0.0), # staging area near origin, faces +X
    "DOCK":    Waypoint("DOCK",    x=0.5, y=0.5, yaw=-math.pi / 2),  # charging dock, faces -Y (south)
}


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    # converts a 2-D heading angle (yaw, radians) to a unit quaternion (x, y, z, w)
    # rotation is purely about the Z-axis, so x=0 and y=0; z and w follow half-angle formulas
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _make_pose(wp: Waypoint, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()                    # allocate a new PoseStamped message
    pose.header.frame_id = frame_id         # set the coordinate frame (always "map" in this project)
    pose.pose.position.x = wp.x             # copy the waypoint's X coordinate into the message
    pose.pose.position.y = wp.y             # copy the waypoint's Y coordinate into the message
    pose.pose.position.z = 0.0             # the robot operates in 2-D, so Z is always 0
    qx, qy, qz, qw = _yaw_to_quat(wp.yaw) # convert yaw angle to quaternion components
    pose.pose.orientation.x = qx           # set quaternion X component (always 0 for pure Z-rotation)
    pose.pose.orientation.y = qy           # set quaternion Y component (always 0 for pure Z-rotation)
    pose.pose.orientation.z = qz           # set quaternion Z component (encodes half-angle sine)
    pose.pose.orientation.w = qw           # set quaternion W component (encodes half-angle cosine)
    return pose                             # return the fully populated PoseStamped message


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class MissionExecutorNode(Node):
    """
    Drives the AMR through a configurable waypoint sequence
    using Nav2 BasicNavigator.
    Follows professor's Node pattern: declare params → init pubs → create timer.
    """


    def __init__(self) -> None:
        super().__init__('warehouse_mission_executor')  # register this node with ROS 2 under that name
        
        # ── Parameters (professor's declare_parameter style) ──────────────
        self.declare_parameter('mission_sequence',   value=['A1', 'B2', 'DOCK'])  # ordered list of waypoint names to visit
        self.declare_parameter('auto_return_to_dock', True)                        # if True, append a DOCK step when the sequence doesn't already end there
        self.declare_parameter('map_frame',          'map') 
        self.declare_parameter('nav_timeout_s',      120.0)                        # maximum seconds to wait for any single navigation step
        self.declare_parameter('feedback_interval_s', 5.0)                        # how often (seconds) to log distance-remaining / ETA

        self._sequence  = self.get_parameter('mission_sequence').get_parameter_value().string_array_value   # read and store the waypoint sequence
        self._auto_dock = self.get_parameter('auto_return_to_dock').get_parameter_value().bool_value        # read and store the auto-dock flag
        self._frame     = self.get_parameter('map_frame').get_parameter_value().string_value                # read and store the map frame name
        self.NAV_TIMEOUT_S     = self.get_parameter('nav_timeout_s').get_parameter_value().double_value     # read tunable timeout
        self.FEEDBACK_INTERVAL = self.get_parameter('feedback_interval_s').get_parameter_value().double_value  # read tunable feedback interval
        # ── Publisher (professor's create_publisher pattern) ───────────────
        self._status_pub = self.create_publisher(String, '/amr/mission_status', 10)  # publishes status strings after each step; queue depth = 10

        # ── Nav2 navigator ─────────────────────────────────────────────────
        self._nav = BasicNavigator()        # create the Nav2 client that sends navigation goals

        self._reports: list[MissionReport] = []  # accumulates one MissionReport per completed step

        self.get_logger().info(f'Mission executor ready — sequence: {self._sequence}')  # log the loaded sequence so operators can verify

    # ── Public API ────────────────────────────────────────────────────────

    def run_sequence(self) -> None:
        """Execute the full mission sequence — called once from main()."""
        self.get_logger().info('─── Mission sequence starting ───')   # banner log so it's easy to spot in a terminal
        self._nav.waitUntilNav2Active()                                 # block here until every Nav2 lifecycle node is in the "active" state

        for step in self._sequence:                                     # iterate over each waypoint name in the configured sequence
            report = self._go(step)                                     # navigate to this waypoint and get a result report
            self._reports.append(report)                                # save the report for the final summary
            self._pub_status(report)                                    # broadcast the outcome on /amr/mission_status
            if report.status == MissionStatus.FAILED:                   # if navigation failed, stop the whole mission
                self.get_logger().error(f"Step '{step}' FAILED — aborting.")  # log the failure with the offending step name
                break                                                   # exit the loop; remaining steps are skipped

        # Auto-dock if last waypoint wasn't already DOCK
        if self._auto_dock and self._sequence[-1].upper() != 'DOCK':   # check: auto-dock is enabled AND the last step wasn't DOCK
            report = self._go('DOCK')                                   # navigate to the charging dock
            self._reports.append(report)                                # record the dock result
            self._pub_status(report)                                    # publish the dock status

        self._print_summary()                                           # print a table of all step outcomes to the log

    # ── Navigation ────────────────────────────────────────────────────────

    def _go(self, name: str) -> MissionReport:
        """Navigate to a named waypoint. Mirrors professor's publish_cmd simplicity."""
        key = name.upper()                  # normalise to upper-case so "a1" and "A1" both work
        if key not in SHELF_REGISTRY:       # guard: reject unknown waypoint names immediately
            msg = f"Unknown waypoint '{key}'"           # build a descriptive error string
            self.get_logger().error(msg)                # log it so it's visible in the terminal
            return MissionReport(key, MissionStatus.FAILED, 0.0, msg)  # return failure report without ever calling Nav2

        wp = SHELF_REGISTRY[key]            # look up the Waypoint object for this key
        self.get_logger().info(
            f'▶ → {wp.name}  ({wp.x:.2f}, {wp.y:.2f}, '
            f'yaw={math.degrees(wp.yaw):.1f}°)'         # log destination coordinates and heading in degrees for readability
        )

        pose = _make_pose(wp, self._frame)              # build the PoseStamped goal message
        pose.header.stamp = self._nav.get_clock().now().to_msg()  # stamp with current sim/wall time so Nav2 accepts the goal
        self._nav.goToPose(pose)                        # send the navigation goal to Nav2 (non-blocking)

        start    = time.monotonic()                     # record the wall-clock start time for elapsed-time tracking
        last_log = start                                # also used to throttle progress-feedback log messages

        while not self._nav.isTaskComplete():           # poll Nav2 until the goal finishes (succeeded, failed, or cancelled)
            elapsed = time.monotonic() - start          # compute how long we have been waiting

            if elapsed > self.NAV_TIMEOUT_S:                                        # check: have we exceeded the maximum allowed time?
                self.get_logger().warn(f"Timeout navigating to '{wp.name}'. Cancelling.")  # warn the operator about the timeout
                self._nav.cancelTask()                                              # send a cancel request to Nav2
                return MissionReport(wp.name, MissionStatus.CANCELLED, elapsed, 'Timeout')  # return immediately with CANCELLED status

            if time.monotonic() - last_log >= self.FEEDBACK_INTERVAL:  # check: time to emit a progress log?
                fb = self._nav.getFeedback()                            # query Nav2 for the latest feedback message
                if fb:                                                  # feedback may be None if Nav2 hasn't sent one yet
                    eta  = Duration.from_msg(fb.estimated_time_remaining).nanoseconds / 1e9  # convert ROS Duration to plain seconds
                    dist = fb.distance_remaining                        # straight-line distance left to goal (metres)
                    self.get_logger().info(f'  {wp.name}: {dist:.2f}m left, ETA {eta:.1f}s')  # log progress in a human-friendly way
                last_log = time.monotonic()                             # reset the log timer

            time.sleep(0.1)                             # yield the CPU for 100 ms to avoid busy-spinning

        elapsed = time.monotonic() - start              # compute total time spent on this navigation step
        result  = self._nav.getResult()                 # retrieve the final TaskResult enum from Nav2

        if result == TaskResult.SUCCEEDED:              # Nav2 confirmed the robot reached the goal within tolerance
            self.get_logger().info(f'✔ {wp.name} reached in {elapsed:.1f}s')   # log the success with timing
            return MissionReport(wp.name, MissionStatus.SUCCEEDED, elapsed)     # return success report
        elif result == TaskResult.CANCELED:             # Nav2 confirms the task was cancelled (e.g. by our timeout handler)
            return MissionReport(wp.name, MissionStatus.CANCELLED, elapsed, 'Cancelled')  # return cancellation report
        else:                                           # any other result (FAILED, unknown) is treated as a failure
            self.get_logger().error(f'✘ {wp.name} FAILED after {elapsed:.1f}s')    # log the failure with timing
            return MissionReport(wp.name, MissionStatus.FAILED, elapsed, 'Nav2 failed')  # return failure report

    # ── Publishers ────────────────────────────────────────────────────────

    def _pub_status(self, r: MissionReport) -> None:
        msg = String()                                              # allocate a new String ROS message
        msg.data = f'{r.status.name}:{r.waypoint}:{r.elapsed_sec:.1f}s'  # format: STATUS:WAYPOINT:TIMEs
        if r.message:                                               # only append the detail field if it is non-empty
            msg.data += f':{r.message}'                            # append optional detail (e.g. ":Timeout" or ":Nav2 failed")
        self._status_pub.publish(msg)                              # send the status string to /amr/mission_status

    # ── Summary ───────────────────────────────────────────────────────────

    def _print_summary(self) -> None:
        self.get_logger().info('─── Mission Summary ───')          # print a section header to separate the summary from per-step logs
        total = 0.0                                                 # accumulator for total mission time in seconds
        for r in self._reports:                                     # iterate over every stored MissionReport
            icon = '✔' if r.status == MissionStatus.SUCCEEDED else '✘'  # pick a checkmark or X based on outcome
            self.get_logger().info(
                f'  {icon} {r.waypoint:<10} {r.status.name:<12} {r.elapsed_sec:6.1f}s'
                + (f'  ({r.message})' if r.message else '')         # append detail message only when present
            )
            total += r.elapsed_sec                                  # add this step's time to the running total
        ok = sum(1 for r in self._reports if r.status == MissionStatus.SUCCEEDED)  # count successful steps
        self.get_logger().info(f'  {ok}/{len(self._reports)} succeeded | {total:.1f}s total')  # final score and total mission time


# ─────────────────────────────────────────────────────────────────────────────
# Entry point — professor's main() pattern
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)                   # initialise the ROS 2 runtime and parse CLI arguments
    node = MissionExecutorNode()            # construct the node (declares params, creates publisher, creates navigator)
    try:
        node.run_sequence()                 # execute the full waypoint sequence (blocks until done)
    except KeyboardInterrupt:               # user pressed Ctrl-C
        node.get_logger().info('Interrupted — cancelling task.')  # log the interrupt so the operator knows why it stopped
        node._nav.cancelTask()              # ask Nav2 to cancel any in-flight navigation goal before we exit
    finally:
        node.destroy_node()                 # release all ROS 2 resources (subscriptions, publishers, timers)
        rclpy.shutdown()                    # shut down the ROS 2 runtime cleanly


if __name__ == '__main__':                  # only run main() when this script is executed directly (not imported)
    main()                                  # call the entry-point function
