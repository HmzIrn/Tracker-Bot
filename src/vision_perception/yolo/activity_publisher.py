"""Wall-clock scheduling state for the /person_activity contract."""

import math
from typing import Optional


VALID_ACTIVITY_LABELS = frozenset(("Walking", "Running", "Staying"))
MIN_ACTIVITY_PUBLISH_HZ = 1.5
MAX_ACTIVITY_PUBLISH_HZ = 10.0
MAX_REPEAT_INTERVAL_SEC = 0.6
MIN_PUBLISH_INTERVAL_SEC = 0.1
INFERENCE_STALE_SEC = 3.0


class ActivityPublisherState:
    def __init__(self, activity_publish_hz: float = 2.0):
        requested_hz = float(activity_publish_hz)
        if not math.isfinite(requested_hz):
            raise ValueError("activity_publish_hz must be finite")
        self.activity_publish_hz = min(
            MAX_ACTIVITY_PUBLISH_HZ,
            max(MIN_ACTIVITY_PUBLISH_HZ, requested_hz),
        )
        self.repeat_interval_sec = min(
            1.0 / self.activity_publish_hz,
            MAX_REPEAT_INTERVAL_SEC,
        )
        self.current_label = ""
        self.last_inference_wall_time: Optional[float] = None
        self.last_publish_wall_time: Optional[float] = None
        self.pending_label_change = False

    def update(self, label: str, wall_time: float) -> Optional[str]:
        if (
            self.last_inference_wall_time is not None
            and wall_time - self.last_inference_wall_time >= INFERENCE_STALE_SEC
        ):
            self.current_label = ""
            self.pending_label_change = False

        self.last_inference_wall_time = wall_time
        valid_label = label if label in VALID_ACTIVITY_LABELS else ""
        changed = valid_label != self.current_label
        self.current_label = valid_label

        if not valid_label:
            self.pending_label_change = False
            return None
        if not changed:
            return None
        if self._publish_interval_elapsed(wall_time):
            self._record_publish(wall_time)
            return valid_label

        self.pending_label_change = True
        return None

    def heartbeat(self, wall_time: float) -> Optional[str]:
        if not self.current_label or self.last_inference_wall_time is None:
            return None
        if wall_time - self.last_inference_wall_time >= INFERENCE_STALE_SEC:
            self.current_label = ""
            self.pending_label_change = False
            return None
        if not self._publish_interval_elapsed(wall_time):
            return None
        if not self.pending_label_change and (
            self.last_publish_wall_time is not None
            and wall_time - self.last_publish_wall_time < self.repeat_interval_sec
        ):
            return None

        label = self.current_label
        self.pending_label_change = False
        self._record_publish(wall_time)
        return label

    def _publish_interval_elapsed(self, wall_time: float) -> bool:
        return (
            self.last_publish_wall_time is None
            or wall_time - self.last_publish_wall_time >= MIN_PUBLISH_INTERVAL_SEC
        )

    def _record_publish(self, wall_time: float) -> None:
        self.last_publish_wall_time = wall_time
