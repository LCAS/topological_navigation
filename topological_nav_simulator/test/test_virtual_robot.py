# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""Tests for virtual_robot module."""

import math

import pytest
geometry_msgs = pytest.importorskip('geometry_msgs', reason='geometry_msgs not available')
from geometry_msgs.msg import Pose, Quaternion  # noqa: E402
from topological_nav_simulator.virtual_robot import (
    _euler_from_quaternion,
    _lerp,
    _quaternion_from_yaw,
    _slerp_yaw,
)


def test_quaternion_from_yaw_zero():
    """Yaw=0 should produce identity-like quaternion."""
    q = _quaternion_from_yaw(0.0)
    assert abs(q.w - 1.0) < 1e-6
    assert abs(q.z) < 1e-6


def test_quaternion_from_yaw_90():
    """Yaw=pi/2 should produce z~0.707, w~0.707."""
    q = _quaternion_from_yaw(math.pi / 2.0)
    assert abs(q.z - math.sin(math.pi / 4.0)) < 1e-6
    assert abs(q.w - math.cos(math.pi / 4.0)) < 1e-6


def test_euler_from_quaternion_roundtrip():
    """Converting yaw to quaternion and back should preserve the value."""
    for yaw in [0.0, math.pi / 4, -math.pi / 3, math.pi, -math.pi / 2]:
        q = _quaternion_from_yaw(yaw)
        recovered = _euler_from_quaternion(q)
        diff = abs(math.atan2(math.sin(yaw - recovered), math.cos(yaw - recovered)))
        assert diff < 1e-6, f'Yaw {yaw} roundtrip failed: got {recovered}'


def test_lerp():
    """Linear interpolation basic checks."""
    assert abs(_lerp(0.0, 10.0, 0.0) - 0.0) < 1e-9
    assert abs(_lerp(0.0, 10.0, 1.0) - 10.0) < 1e-9
    assert abs(_lerp(0.0, 10.0, 0.5) - 5.0) < 1e-9


def test_slerp_yaw():
    """Yaw interpolation basic checks."""
    assert abs(_slerp_yaw(0.0, math.pi, 0.0) - 0.0) < 1e-6
    r = _slerp_yaw(0.0, math.pi / 2, 0.5)
    assert abs(r - math.pi / 4) < 1e-6


def test_slerp_yaw_wraparound():
    """Slerp should take the short path around +-pi."""
    # From -170 to 170 degrees should go through 180, not through 0
    a = math.radians(-170)
    b = math.radians(170)
    mid = _slerp_yaw(a, b, 0.5)
    # Should be near +-180 degrees
    assert abs(abs(mid) - math.pi) < math.radians(5)
