"""Goal pose manager and free-space navigation helper.

Handles Staying dwell timing (>=5.0s), once-per-episode goal emission,
global cooldown gating (>=15.0s), world-to-map frame transformation,
and occupancy grid free-space nudging.
"""

import math
from typing import List, Optional, Sequence, Set, Tuple


def image_stamp_to_seconds(stamp) -> float:
    """Convert a ROS image header stamp to seconds without consulting node time."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def transform_world_to_map(
    wx: float, wy: float, world_to_map: Sequence[float]
) -> Tuple[float, float]:
    """Transform 2D point from Gazebo world frame to Cartographer map frame.

    Args:
        wx: X in world frame (meters).
        wy: Y in world frame (meters).
        world_to_map: [tx, ty, yaw_radians].

    Returns:
        (mx, my) in map frame (meters).
    """
    tx, ty, yaw = float(world_to_map[0]), float(world_to_map[1]), float(world_to_map[2])
    c = math.cos(yaw)
    s = math.sin(yaw)
    mx = tx + (c * wx - s * wy)
    my = ty + (s * wx + c * wy)
    return float(mx), float(my)


def nudge_to_free_space(
    target_x: float,
    target_y: float,
    grid_data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    max_radius: float = 0.5,
) -> Optional[Tuple[float, float]]:
    """Check if (target_x, target_y) is free (cell == 0). If not, find nearest free cell within max_radius.

    Args:
        target_x: Target X in map frame (meters).
        target_y: Target Y in map frame (meters).
        grid_data: 1D occupancy grid data (-1: unknown, 0: free, 100: occupied).
        width: Grid width in cells.
        height: Grid height in cells.
        resolution: Grid resolution (meters/cell).
        origin_x: Map origin X (meters).
        origin_y: Map origin Y (meters).
        max_radius: Maximum search radius in meters (default 0.5m).

    Returns:
        (nudge_x, nudge_y) if a free cell is found within max_radius, else None.
    """
    if width <= 0 or height <= 0 or resolution <= 0:
        return None

    # Target continuous cell coordinates
    target_ix = int(math.floor((target_x - origin_x) / resolution))
    target_iy = int(math.floor((target_y - origin_y) / resolution))

    # Helper to check cell value
    def is_free(ix: int, iy: int) -> bool:
        if 0 <= ix < width and 0 <= iy < height:
            idx = iy * width + ix
            if 0 <= idx < len(grid_data):
                return grid_data[idx] == 0
        return False

    # Check target cell directly
    if is_free(target_ix, target_iy):
        return target_x, target_y

    # Search outward in radial circle up to max_radius
    cell_radius = int(math.ceil(max_radius / resolution))
    best_pt = None
    best_dist_sq = max_radius * max_radius + 1e-6

    for dy in range(-cell_radius, cell_radius + 1):
        for dx in range(-cell_radius, cell_radius + 1):
            ix = target_ix + dx
            iy = target_iy + dy
            if is_free(ix, iy):
                # Center of this free cell in map coordinates
                cell_world_x = origin_x + (ix + 0.5) * resolution
                cell_world_y = origin_y + (iy + 0.5) * resolution
                dist_sq = (cell_world_x - target_x) ** 2 + (cell_world_y - target_y) ** 2
                if dist_sq <= max_radius * max_radius and dist_sq < best_dist_sq:
                    best_dist_sq = dist_sq
                    best_pt = (float(cell_world_x), float(cell_world_y))

    return best_pt


class GoalPoseManager:
    """Tracks Staying dwell duration and emits goal triggers adhering to contract."""

    def __init__(
        self,
        staying_dwell_sec: float = 5.0,
        goal_cooldown_sec: float = 15.0,
        world_to_map: Sequence[float] = (0.0, 0.0, 0.0),
    ):
        self.staying_dwell_sec = float(staying_dwell_sec)
        self.goal_cooldown_sec = float(goal_cooldown_sec)
        self.world_to_map = list(world_to_map)

        # track_id -> {"start_time": float, "positions": List[(x,y)], "fired": bool}
        self.episodes = {}
        self.last_goal_time: float = -1e9

    def update_track(
        self,
        track_id: int,
        activity: str,
        world_pos: Tuple[float, float],
        sim_time: float,
    ) -> Optional[Tuple[float, float]]:
        """Update track state. Returns candidate (map_x, map_y) if goal should fire, else None."""
        if activity == "Staying":
            if track_id not in self.episodes:
                self.episodes[track_id] = {
                    "start_time": sim_time,
                    "positions": [world_pos],
                    "fired": False,
                }
            else:
                self.episodes[track_id]["positions"].append(world_pos)

            ep = self.episodes[track_id]
            dwell = sim_time - ep["start_time"]

            if dwell >= self.staying_dwell_sec and not ep["fired"]:
                # Check global cooldown
                if (sim_time - self.last_goal_time) >= self.goal_cooldown_sec:
                    # Calculate median floor position
                    positions = ep["positions"]
                    xs = sorted([p[0] for p in positions])
                    ys = sorted([p[1] for p in positions])
                    mid = len(xs) // 2
                    med_x = xs[mid] if len(xs) % 2 != 0 else (xs[mid - 1] + xs[mid]) / 2.0
                    med_y = ys[mid] if len(ys) % 2 != 0 else (ys[mid - 1] + ys[mid]) / 2.0

                    map_x, map_y = transform_world_to_map(med_x, med_y, self.world_to_map)
                    return map_x, map_y
        else:
            # Leaving Staying resets episode
            if track_id in self.episodes:
                del self.episodes[track_id]

        return None

    def mark_fired(self, track_id: int, sim_time: float):
        """Mark goal as published for current episode of track_id."""
        if track_id in self.episodes:
            self.episodes[track_id]["fired"] = True
        self.last_goal_time = sim_time

    def cleanup(self, active_track_ids: Set[int]):
        """Drop episodes for tracks no longer active."""
        for tid in list(self.episodes.keys()):
            if tid not in active_track_ids:
                del self.episodes[tid]
