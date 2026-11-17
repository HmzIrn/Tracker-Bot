"""Unit tests for ground projection onto the z=0 floor plane."""

import math
import os
import sys

# Ensure local module directory is in sys.path
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from ground_projection import project_pixel_to_ground, GroundProjector


def test_center_ray_pitch_distance():
    """Verify that image center ray with pitch theta hits floor at z_c / tan(theta)."""
    pitch = 0.25
    zc = 2.0
    pose = [0.0, 0.0, zc, 0.0, pitch, 0.0]  # Facing along +X
    # Center pixel of 640x480
    res = project_pixel_to_ground(320.0, 240.0, pose, width=640, height=480, hfov=1.05)
    assert res is not None
    x, y = res

    expected_dist = zc / math.tan(pitch)
    assert abs(x - expected_dist) < 1e-3
    assert abs(y - 0.0) < 1e-3


def test_camera1_center_projection():
    """Verify camera 1 center ray hits world coordinate (-0.631, -0.631)."""
    pose1 = [-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854]
    res = project_pixel_to_ground(320.0, 240.0, pose1, width=640, height=480, hfov=1.05)
    assert res is not None
    x, y = res

    expected_offset = (2.3 / math.tan(0.25)) * math.cos(math.pi / 4.0)
    expected_x = -7.0 + expected_offset
    expected_y = -7.0 + expected_offset

    assert abs(x - expected_x) < 0.02
    assert abs(y - expected_y) < 0.02


def test_camera2_center_projection():
    """Verify camera 2 center ray follows its simulated north-side mounting pose."""
    pose2 = [0.0, 7.0, 2.3, 0.0, 0.2, -1.57]
    res = project_pixel_to_ground(320.0, 240.0, pose2, width=640, height=480, hfov=1.05)
    assert res is not None
    x, y = res

    expected_x = 0.0
    expected_y = 7.0 - (2.3 / math.tan(0.2))

    assert abs(x - expected_x) < 0.02
    assert abs(y - expected_y) < 0.02


def test_sky_ray_returns_none():
    """Verify that a ray pointing above the horizon does not intersect the ground."""
    # Horizontal camera (pitch=0), top pixel (v=0) points upward into the sky
    pose = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
    res = project_pixel_to_ground(320.0, 0.0, pose, width=640, height=480, hfov=1.05)
    assert res is None


def test_ground_projector_class():
    """Verify GroundProjector class handles multi-camera config."""
    configs = {
        "cam1": {"pose": [-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854], "width": 640, "height": 480, "hfov": 1.05},
        "cam2": {"pose": [7.0, 7.0, 2.3, 0.0, 0.25, -2.356], "width": 640, "height": 480, "hfov": 1.05},
    }
    projector = GroundProjector(configs)
    p1 = projector.project("cam1", 320.0, 240.0)
    p2 = projector.project("cam2", 320.0, 240.0)
    assert p1 is not None and p2 is not None
    assert p1[0] < 0.0 and p2[0] > 0.0
