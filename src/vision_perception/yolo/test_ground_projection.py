"""Unit tests for camera-configured horizontal-plane projection."""

import math
import os
import sys

import pytest

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
    """Verify corner camera center ray hits world coordinate (-0.631, -0.631)."""
    pose1 = [-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854]
    res = project_pixel_to_ground(320.0, 240.0, pose1, width=640, height=480, hfov=1.05)
    assert res is not None
    x, y = res

    expected_offset = (2.3 / math.tan(0.25)) * math.cos(math.pi / 4.0)
    expected_x = -7.0 + expected_offset
    expected_y = -7.0 + expected_offset

    assert abs(x - expected_x) < 0.02
    assert abs(y - expected_y) < 0.02


def test_overhead_camera_projection():
    """Verify overhead camera (pose [0, 0, 12.2, 0, 1.5708, 1.5708]) center hits (0, 0)."""
    pose_overhead = [0.0, 0.0, 12.2, 0.0, 1.5708, 1.5708]
    res = project_pixel_to_ground(320.0, 240.0, pose_overhead, width=640, height=480, hfov=1.50)
    assert res is not None
    x, y = res
    assert abs(x - 0.0) < 1e-3
    assert abs(y - 0.0) < 1e-3

    # Off-center: pixel right (u=640, v=240) should give x > 0, y = 0
    res_right = project_pixel_to_ground(640.0, 240.0, pose_overhead, width=640, height=480, hfov=1.50)
    assert res_right is not None
    assert res_right[0] > 0.0
    assert abs(res_right[1] - 0.0) < 1e-3

    # Pixel up (u=320, v=0) should give x = 0, y > 0
    res_up = project_pixel_to_ground(320.0, 0.0, pose_overhead, width=640, height=480, hfov=1.50)
    assert res_up is not None
    assert abs(res_up[0] - 0.0) < 1e-3
    assert res_up[1] > 0.0


def test_nonzero_projection_plane_and_waypoint_round_trips():
    """Check overhead plane_z and map-coordinate error at the center and a wall waypoint."""
    configs = {
        "overhead": {
            "pose": [0.0, 0.0, 12.2, 0.0, 1.5708, 1.5708],
            "width": 640, "height": 480, "hfov": 1.50,
            "plane_z": 1.5, "ground_point": "center",
        },
        "corner": {
            "pose": [-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854],
            "width": 640, "height": 480, "hfov": 1.05,
            "plane_z": 0.0, "ground_point": "bottom_center",
        },
    }
    projector = GroundProjector(configs)
    test_points = (
        ("overhead", (0.0, 0.0, 1.5)),
        ("corner", (0.5, 4.5, 0.0)),
    )
    errors = []
    for cam_id, (x, y, z) in test_points:
        pixel = projector.project_world_to_pixel(cam_id, x, y, z)
        assert pixel is not None
        estimate = projector.project(cam_id, *pixel)
        assert estimate is not None
        error = math.hypot(estimate[0] - x, estimate[1] - y)
        errors.append(error)
        assert error < 0.1

    assert errors[0] == pytest.approx(0.0, abs=1e-3)


def test_sky_ray_returns_none():
    """Verify that a ray pointing above the horizon does not intersect the ground."""
    # Horizontal camera (pitch=0), top pixel (v=0) points upward into the sky
    pose = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
    res = project_pixel_to_ground(320.0, 0.0, pose, width=640, height=480, hfov=1.05)
    assert res is None


def test_ground_projector_class():
    """Verify GroundProjector class handles multi-camera config and modes."""
    configs = {
        "overhead": {
            "pose": [0.0, 0.0, 12.2, 0.0, 1.5708, 1.5708],
            "width": 640,
            "height": 480,
            "hfov": 1.50,
            "plane_z": 1.5,
            "ground_point": "center",
        },
        "corner": {
            "pose": [-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854],
            "width": 640,
            "height": 480,
            "hfov": 1.05,
            "plane_z": 0.0,
            "ground_point": "bottom_center",
        },
    }
    projector = GroundProjector(configs)
    assert projector.get_ground_point_type("overhead") == "center"
    assert projector.get_ground_point_type("corner") == "bottom_center"

    p_center = projector.project_bbox("overhead", [300, 220, 340, 260])
    assert p_center is not None
    assert abs(p_center[0]) < 1e-3 and abs(p_center[1]) < 1e-3
