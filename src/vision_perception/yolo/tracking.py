"""Time-aware nearest-neighbor association for projected person detections."""

import math
from typing import Dict, List, Sequence, Tuple


Position = Tuple[float, float]
TrackObservation = Tuple[Position, float]
AssociationCandidate = Tuple[float, int, int]


def association_candidates(
    detection_positions: Sequence[Position],
    tracks: Dict[int, TrackObservation],
    current_time: float,
    minimum_distance: float,
    maximum_speed_mps: float = 4.0,
) -> List[AssociationCandidate]:
    """Return distance-sorted detection/track matches within an age-scaled gate."""
    candidates = []
    for detection_index, detection_position in enumerate(detection_positions):
        for track_id, (track_position, last_seen) in tracks.items():
            age = max(0.0, current_time - last_seen)
            gate = max(minimum_distance, maximum_speed_mps * age)
            distance = math.hypot(
                detection_position[0] - track_position[0],
                detection_position[1] - track_position[1],
            )
            if distance <= gate:
                candidates.append((distance, detection_index, track_id))

    candidates.sort(key=lambda candidate: candidate[0])
    return candidates
