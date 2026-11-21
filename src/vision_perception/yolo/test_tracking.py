"""Tests for motion-aware global-track association."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from tracking import association_candidates


def test_association_gate_grows_with_track_age():
    position = (0.0, 0.0)
    tracks = {1: (position, 0.0)}

    short_gap = association_candidates(
        [(1.6, 0.0)], tracks, current_time=0.2, minimum_distance=1.5
    )
    long_gap = association_candidates(
        [(1.6, 0.0)], tracks, current_time=0.6, minimum_distance=1.5
    )

    assert short_gap == []
    assert long_gap == [(1.6, 0, 1)]


def test_three_mps_replay_at_measured_update_rate_keeps_track_id():
    update_rate_hz = 2.47
    dt = 1.0 / update_rate_hz
    previous_position = (-0.2, 0.0)
    tracks = {1: (previous_position, 0.0)}
    track_id = 1
    restarts = 0

    for sample in range(1, 21):
        timestamp = sample * dt
        measurement_jitter = 0.2 if sample % 2 else -0.2
        position = (3.0 * timestamp + measurement_jitter, 0.0)
        candidates = association_candidates(
            [position],
            tracks,
            current_time=timestamp,
            minimum_distance=1.5,
        )

        if candidates:
            track_id = candidates[0][2]
        else:
            restarts += 1
            track_id += 1

        tracks = {track_id: (position, timestamp)}

    assert restarts == 0
    assert track_id == 1
