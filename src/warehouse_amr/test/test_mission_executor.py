#!/usr/bin/env python3
"""
test_mission_executor.py
────────────────────────
Unit tests for the pure-Python helpers in mission_executor.py.
These tests do NOT require a running ROS 2 instance — they test
only the math and data-structure logic that has no ROS dependencies.

Run with:
    cd ~/ros2_ws
    source install/setup.bash
    colcon test --packages-select warehouse_amr
    colcon test-result --verbose
"""

from __future__ import annotations

import math
import os
import sys

import pytest

# ── Import the helpers directly without initialising ROS ──────────────────────
# mission_executor.py uses 'from __future__ import annotations' and imports ROS
# packages at module level.  We mock the ROS modules before importing so that
# the pure-Python helpers (_yaw_to_quat, _make_pose, SHELF_REGISTRY, Waypoint)
# can be tested without a live ROS 2 environment.

import types

# Minimal stubs for every ROS module that mission_executor imports at the top
for mod_name in [
    "rclpy",
    "rclpy.node",
    "rclpy.duration",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_simple_commander",
    "nav2_simple_commander.robot_navigator",
    "std_msgs",
    "std_msgs.msg",
]:
    sys.modules.setdefault(mod_name, types.ModuleType(mod_name))

# Stub the specific symbols mission_executor references at import time
sys.modules["rclpy.node"].Node = object
sys.modules["rclpy.duration"].Duration = object
sys.modules["nav2_simple_commander.robot_navigator"].BasicNavigator = object
sys.modules["nav2_simple_commander.robot_navigator"].TaskResult = object

# PoseStamped stub — _make_pose() constructs one of these
class _PoseStamped:
    class _Header:
        frame_id: str = ""
    class _Pose:
        class _Position:
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0
        class _Orientation:
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0
            w: float = 0.0
        position = _Position()
        orientation = _Orientation()
    header = _Header()
    pose = _Pose()

sys.modules["geometry_msgs.msg"].PoseStamped = _PoseStamped
sys.modules["std_msgs.msg"].String = object

# Now we can safely import the pure helpers
scripts_dir = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(scripts_dir))
from mission_executor import _yaw_to_quat, _make_pose, SHELF_REGISTRY, Waypoint


# ─────────────────────────────────────────────────────────────────────────────
# _yaw_to_quat
# ─────────────────────────────────────────────────────────────────────────────

class TestYawToQuat:
    """Tests for the yaw → quaternion conversion helper."""

    def test_zero_yaw_gives_identity_quaternion(self):
        """yaw=0 → (x=0, y=0, z=0, w=1): no rotation, facing +X."""
        qx, qy, qz, qw = _yaw_to_quat(0.0)
        assert qx == pytest.approx(0.0)
        assert qy == pytest.approx(0.0)
        assert qz == pytest.approx(0.0)
        assert qw == pytest.approx(1.0)

    def test_pi_yaw_gives_180_degree_rotation(self):
        """yaw=π → (x=0, y=0, z=1, w=0): 180° around Z, facing −X (aisle B)."""
        qx, qy, qz, qw = _yaw_to_quat(math.pi)
        assert qx == pytest.approx(0.0, abs=1e-9)
        assert qy == pytest.approx(0.0, abs=1e-9)
        assert qz == pytest.approx(1.0, abs=1e-9)
        assert qw == pytest.approx(0.0, abs=1e-9)

    def test_minus_half_pi_gives_dock_heading(self):
        """yaw=−π/2 → facing −Y (south): the DOCK waypoint heading."""
        qx, qy, qz, qw = _yaw_to_quat(-math.pi / 2)
        assert qx == pytest.approx(0.0, abs=1e-9)
        assert qy == pytest.approx(0.0, abs=1e-9)
        assert qz == pytest.approx(-math.sqrt(2) / 2, abs=1e-9)
        assert qw == pytest.approx(math.sqrt(2) / 2, abs=1e-9)

    def test_output_is_always_unit_norm(self):
        """||q|| must equal 1.0 for every input angle."""
        angles = [0.0, math.pi / 6, math.pi / 4, math.pi / 2,
                  math.pi, -math.pi / 2, -math.pi / 4, 2 * math.pi]
        for yaw in angles:
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            assert norm == pytest.approx(1.0, abs=1e-9), (
                f"Non-unit quaternion at yaw={yaw:.4f}: norm={norm}"
            )

    def test_x_and_y_components_always_zero(self):
        """Pure Z-rotation quaternions always have qx=0 and qy=0."""
        for yaw in [0.0, 0.5, 1.0, math.pi, -1.0]:
            qx, qy, _, _ = _yaw_to_quat(yaw)
            assert qx == pytest.approx(0.0)
            assert qy == pytest.approx(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# _make_pose
# ─────────────────────────────────────────────────────────────────────────────

class TestMakePose:
    """Tests for the Waypoint → PoseStamped conversion helper."""

    def _make_wp(self, x=2.0, y=3.5, yaw=0.0) -> Waypoint:
        return Waypoint(name="TEST", x=x, y=y, yaw=yaw)

    def test_frame_id_is_set_correctly(self):
        pose = _make_pose(self._make_wp(), frame_id="map")
        assert pose.header.frame_id == "map"

    def test_position_x_and_y_are_copied(self):
        pose = _make_pose(self._make_wp(x=5.0, y=6.5))
        assert pose.pose.position.x == pytest.approx(5.0)
        assert pose.pose.position.y == pytest.approx(6.5)

    def test_position_z_is_always_zero(self):
        """Robot operates in 2-D — Z must always be 0."""
        pose = _make_pose(self._make_wp())
        assert pose.pose.position.z == pytest.approx(0.0)

    def test_orientation_matches_yaw_to_quat(self):
        """Orientation quaternion must match _yaw_to_quat output exactly."""
        yaw = math.pi / 3
        pose = _make_pose(self._make_wp(yaw=yaw))
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        assert pose.pose.orientation.x == pytest.approx(qx)
        assert pose.pose.orientation.y == pytest.approx(qy)
        assert pose.pose.orientation.z == pytest.approx(qz)
        assert pose.pose.orientation.w == pytest.approx(qw)

    def test_default_frame_is_map(self):
        pose = _make_pose(self._make_wp())
        assert pose.header.frame_id == "map"


# ─────────────────────────────────────────────────────────────────────────────
# SHELF_REGISTRY
# ─────────────────────────────────────────────────────────────────────────────

class TestShelfRegistry:
    """Tests for the waypoint name → Waypoint registry."""

    EXPECTED_KEYS = {"A1", "A2", "A3", "B1", "B2", "B3",
                     "C1", "C2", "C3", "STAGING", "DOCK"}

    def test_all_11_waypoints_present(self):
        assert set(SHELF_REGISTRY.keys()) == self.EXPECTED_KEYS

    def test_aisle_a_faces_plus_x(self):
        for key in ["A1", "A2", "A3"]:
            assert SHELF_REGISTRY[key].yaw == pytest.approx(0.0), (
                f"{key} should face +X (yaw=0)"
            )

    def test_aisle_b_faces_minus_x(self):
        for key in ["B1", "B2", "B3"]:
            assert SHELF_REGISTRY[key].yaw == pytest.approx(math.pi, abs=1e-9), (
                f"{key} should face −X (yaw=π)"
            )

    def test_aisle_c_faces_plus_x(self):
        for key in ["C1", "C2", "C3"]:
            assert SHELF_REGISTRY[key].yaw == pytest.approx(0.0), (
                f"{key} should face +X (yaw=0)"
            )

    def test_dock_faces_south(self):
        dock = SHELF_REGISTRY["DOCK"]
        assert dock.yaw == pytest.approx(-math.pi / 2, abs=1e-9)

    def test_dock_coordinates(self):
        dock = SHELF_REGISTRY["DOCK"]
        assert dock.x == pytest.approx(0.5)
        assert dock.y == pytest.approx(0.5)

    def test_all_coordinates_non_negative(self):
        for key, wp in SHELF_REGISTRY.items():
            assert wp.x >= 0.0, f"{key}.x is negative"
            assert wp.y >= 0.0, f"{key}.y is negative"

    def test_waypoint_names_match_registry_keys(self):
        """Each Waypoint's .name field must match its registry key."""
        for key, wp in SHELF_REGISTRY.items():
            assert wp.name == key, (
                f"Registry key '{key}' has Waypoint.name='{wp.name}'"
            )

    def test_aisle_x_coordinates(self):
        """A=2.0 m, B=5.0 m, C=8.0 m — must match the SDF world."""
        for key in ["A1", "A2", "A3"]:
            assert SHELF_REGISTRY[key].x == pytest.approx(2.0)
        for key in ["B1", "B2", "B3"]:
            assert SHELF_REGISTRY[key].x == pytest.approx(5.0)
        for key in ["C1", "C2", "C3"]:
            assert SHELF_REGISTRY[key].x == pytest.approx(8.0)