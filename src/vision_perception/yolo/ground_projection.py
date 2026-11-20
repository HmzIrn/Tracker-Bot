"""Ground projection module for surveillance cameras.

Back-projects pixel coordinates (e.g. bounding box bottom-center corresponding to feet
or bbox center for overhead top-down view) onto the ground plane z=0 in Gazebo world coordinates.
"""

import math
from typing import Dict, Optional, Sequence, Tuple


def project_pixel_to_ground(
    u: float,
    v: float,
    camera_pose: Sequence[float],
    width: int = 640,
    height: int = 480,
    hfov: float = 1.05,
) -> Optional[Tuple[float, float]]:
    """Project image pixel (u, v) to ground plane z=0.

    Args:
        u: pixel column (0 to width).
        v: pixel row (0 to height).
        camera_pose: [x, y, z, roll, pitch, yaw] in meters and radians.
        width: image width in pixels.
        height: image height in pixels.
        hfov: horizontal field of view in radians.

    Returns:
        (x_world, y_world) in meters, or None if ray does not intersect z=0 in front.
    """
    xc, yc, zc, roll, pitch, yaw = camera_pose

    # Pinhole intrinsics
    f = (width / 2.0) / math.tan(hfov / 2.0)
    cx = width / 2.0
    cy = height / 2.0

    # Ray in Gazebo camera link sensor frame (+X forward, +Y left, +Z up)
    vx_s = 1.0
    vy_s = -(u - cx) / f
    vz_s = -(v - cy) / f

    # Rotation matrix R = R_z(yaw) * R_y(pitch) * R_x(roll)
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cyaw = math.cos(yaw)
    syaw = math.sin(yaw)

    r00 = cyaw * cp
    r01 = -syaw * cr + cyaw * sp * sr
    r02 = syaw * sr + cyaw * sp * cr

    r10 = syaw * cp
    r11 = cyaw * cr + syaw * sp * sr
    r12 = -cyaw * sr + syaw * sp * cr

    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    vx_w = r00 * vx_s + r01 * vy_s + r02 * vz_s
    vy_w = r10 * vx_s + r11 * vy_s + r12 * vz_s
    vz_w = r20 * vx_s + r21 * vy_s + r22 * vz_s

    # Ray equation: P(t) = P_cam + t * V_w
    # Floor: z = 0 => zc + t * vz_w = 0 => t = -zc / vz_w
    if vz_w >= -1e-6:
        # Ray points horizontal or upward away from ground
        return None

    t = -zc / vz_w
    if t <= 0:
        return None

    xw = xc + t * vx_w
    yw = yc + t * vy_w
    return float(xw), float(yw)


class GroundProjector:
    """Manages ground projection for multiple calibrated cameras."""

    def __init__(self, camera_configs: Dict[str, dict]):
        """Initialize with dictionary of camera configurations."""
        self.cameras = {}
        for cam_id, cfg in camera_configs.items():
            if not isinstance(cfg, dict) or "pose" not in cfg:
                continue
            self.cameras[cam_id] = {
                "pose": cfg["pose"],
                "width": cfg.get("width", 640),
                "height": cfg.get("height", 480),
                "hfov": cfg.get("hfov", 1.05),
                "ground_point": cfg.get("ground_point", "bottom_center"),
            }

    def get_ground_point_type(self, cam_id: str) -> str:
        """Get ground point extraction mode for camera ('bottom_center' or 'center')."""
        if cam_id not in self.cameras:
            raise KeyError(f"Unknown camera ID: {cam_id}")
        return self.cameras[cam_id]["ground_point"]

    def project(self, cam_id: str, u: float, v: float) -> Optional[Tuple[float, float]]:
        """Project pixel (u, v) from camera cam_id to world ground (x, y)."""
        if cam_id not in self.cameras:
            raise KeyError(f"Unknown camera ID: {cam_id}")
        cfg = self.cameras[cam_id]
        return project_pixel_to_ground(
            u,
            v,
            cfg["pose"],
            width=cfg["width"],
            height=cfg["height"],
            hfov=cfg["hfov"],
        )

    def project_bbox(self, cam_id: str, bbox: Sequence[float]) -> Optional[Tuple[float, float]]:
        """Project bbox [x1, y1, x2, y2] to world ground using camera's ground_point mode."""
        x1, y1, x2, y2 = bbox
        mode = self.get_ground_point_type(cam_id)
        if mode == "center":
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0
        else:
            u = (x1 + x2) / 2.0
            v = float(y2)
        return self.project(cam_id, u, v)
