"""Ground projection module for surveillance cameras.

Back-projects pixel coordinates onto a camera-configured horizontal plane in Gazebo
world coordinates.
"""

import math
from typing import Dict, Optional, Sequence, Tuple


def project_pixel_to_ground(
    u: float,
    v: float,
    camera_pose: Sequence[float],
    width: int,
    height: int,
    hfov: float,
    plane_z: float = 0.0,
) -> Optional[Tuple[float, float]]:
    """Project image pixel (u, v) to a horizontal world plane.

    Args:
        u: pixel column (0 to width).
        v: pixel row (0 to height).
        camera_pose: [x, y, z, roll, pitch, yaw] in meters and radians.
        width: image width in pixels.
        height: image height in pixels.
        hfov: horizontal field of view in radians.
        plane_z: World-coordinate height of the camera's projection plane.

    Returns:
        (x_world, y_world) in meters, or None if the ray does not intersect the plane.
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

    # Ray equation: P(t) = P_cam + t * V_w; intersect with z = plane_z.
    if abs(vz_w) < 1e-6:
        return None

    t = (plane_z - zc) / vz_w
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
                "width": int(cfg["width"]),
                "height": int(cfg["height"]),
                "hfov": float(cfg["hfov"]),
                "plane_z": float(cfg["plane_z"]),
                "ground_point": cfg.get("ground_point", "bottom_center"),
            }

    def project_world_to_pixel(
        self, cam_id: str, x: float, y: float, z: float = 0.0
    ) -> Optional[Tuple[int, int]]:
        """Project a world-space point into a camera image."""
        if cam_id not in self.cameras:
            raise KeyError(f"Unknown camera ID: {cam_id}")
        cfg = self.cameras[cam_id]
        xc, yc, zc, roll, pitch, yaw = cfg["pose"]

        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        r00 = cyaw * cp
        r01 = -syaw * cr + cyaw * sp * sr
        r02 = syaw * sr + cyaw * sp * cr
        r10 = syaw * cp
        r11 = cyaw * cr + syaw * sp * sr
        r12 = -cyaw * sr + syaw * sp * cr
        r20 = -sp
        r21 = cp * sr
        r22 = cp * cr

        dx, dy, dz = x - xc, y - yc, z - zc
        vx = r00 * dx + r10 * dy + r20 * dz
        vy = r01 * dx + r11 * dy + r21 * dz
        vz = r02 * dx + r12 * dy + r22 * dz
        if vx <= 1e-6:
            return None

        width, height = cfg["width"], cfg["height"]
        focal = (width / 2.0) / math.tan(cfg["hfov"] / 2.0)
        u = width / 2.0 - focal * vy / vx
        v = height / 2.0 - focal * vz / vx
        return int(round(u)), int(round(v))

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
            plane_z=cfg["plane_z"],
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
