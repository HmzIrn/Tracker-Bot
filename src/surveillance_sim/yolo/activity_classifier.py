"""Activity classification module for multi-frame tracked persons.

Computes metric speed in meters per second (m/s) over a sliding window
in world ground coordinates (x, y) and classifies movement into Staying,
Walking, Running, or Unknown with majority-vote smoothing and track timeout management.
"""

from collections import Counter, deque
import math
from typing import Dict, List, Optional, Tuple


class TrackState:
    """Maintains tracking history and activity state for a single track ID."""

    def __init__(self, smoothing_n: int = 5):
        # List of (timestamp, x, y)
        self.history: List[Tuple[float, float, float]] = []
        self.last_seen: float = 0.0
        self.recent_activities: deque = deque(maxlen=smoothing_n)
        self.current_speed: float = 0.0
        self.current_activity: str = "Unknown"


class ActivityClassifier:
    """ROS-independent sliding-window activity classifier using metric world coordinates."""

    def __init__(
        self,
        window_sec: float = 1.0,
        staying_thresh: float = 0.3,
        running_thresh: float = 1.75,
        smoothing_n: int = 5,
        track_timeout_sec: float = 2.0,
        min_samples: int = 3,
        min_window_ratio: float = 0.25,
    ):
        """Initialize classifier parameters.

        Args:
            window_sec: Sliding window length in seconds for speed estimation.
            staying_thresh: Speed threshold in m/s below which state is 'Staying'.
            running_thresh: Speed threshold in m/s above which state is 'Running'.
            smoothing_n: Window size for majority-vote smoothing.
            track_timeout_sec: Time in seconds after which inactive tracks are dropped.
            min_samples: Minimum historical frames before non-Unknown classification.
            min_window_ratio: Fraction of window_sec required before valid calculation.
        """
        self.window_sec = float(window_sec)
        self.staying_thresh = float(staying_thresh)
        self.running_thresh = float(running_thresh)
        self.smoothing_n = int(smoothing_n)
        self.track_timeout_sec = float(track_timeout_sec)
        self.min_samples = int(min_samples)
        self.min_window_ratio = float(min_window_ratio)

        self.tracks: Dict[int, TrackState] = {}

    def update(
        self,
        track_id: int,
        world_pos: Tuple[float, float],
        timestamp: float,
    ) -> Tuple[str, float]:
        """Update a track with new world ground coordinates (x, y) and timestamp.

        Args:
            track_id: Global track identifier.
            world_pos: (x, y) position on ground plane in meters.
            timestamp: Sim-time timestamp in seconds.

        Returns:
            Tuple of (smoothed_activity, speed_in_mps).
        """
        x, y = float(world_pos[0]), float(world_pos[1])

        if track_id not in self.tracks:
            self.tracks[track_id] = TrackState(smoothing_n=self.smoothing_n)

        track = self.tracks[track_id]
        track.last_seen = timestamp
        track.history.append((timestamp, x, y))

        # Prune points older than (timestamp - window_sec),
        # but keep at least one older point so we span the requested window
        cutoff = timestamp - self.window_sec
        while len(track.history) > 2 and track.history[1][0] < cutoff:
            track.history.pop(0)

        # Calculate speed if minimum history condition is satisfied
        oldest_t, oldest_x, oldest_y = track.history[0]
        dt = timestamp - oldest_t
        min_dt = self.window_sec * self.min_window_ratio

        if len(track.history) < self.min_samples or dt < min_dt or dt <= 0:
            inst_activity = "Unknown"
            speed = 0.0
        else:
            dx = x - oldest_x
            dy = y - oldest_y
            disp = math.hypot(dx, dy)
            speed = disp / dt

            if speed < self.staying_thresh:
                inst_activity = "Staying"
            elif speed < self.running_thresh:
                inst_activity = "Walking"
            else:
                inst_activity = "Running"

        track.current_speed = speed
        track.recent_activities.append(inst_activity)

        # Majority vote smoothing
        valid_activities = [a for a in track.recent_activities if a != "Unknown"]
        if not valid_activities:
            smoothed = "Unknown"
        else:
            counts = Counter(valid_activities)
            top_count = max(counts.values())
            candidates = [act for act, cnt in counts.items() if cnt == top_count]
            if len(candidates) == 1:
                smoothed = candidates[0]
            else:
                for a in reversed(track.recent_activities):
                    if a in candidates:
                        smoothed = a
                        break
                else:
                    smoothed = candidates[0]

        track.current_activity = smoothed
        return smoothed, speed

    def cleanup(self, current_timestamp: float) -> List[int]:
        """Drop tracks that haven't been updated within track_timeout_sec."""
        dropped = []
        for tid, track in list(self.tracks.items()):
            if current_timestamp - track.last_seen > self.track_timeout_sec:
                dropped.append(tid)
                del self.tracks[tid]
        return dropped

    def get_track_info(self, track_id: int) -> Optional[Tuple[str, float]]:
        """Return (activity, speed) for a track if present."""
        if track_id in self.tracks:
            t = self.tracks[track_id]
            return t.current_activity, t.current_speed
        return None

    def reset(self):
        """Clear all tracks."""
        self.tracks.clear()
