"""Unit tests for ActivityClassifier using synthetic metric tracks.

Tests:
1. Fewer than min_samples returns Unknown.
2. Stationary track -> Staying (speed ~0.0 m/s).
3. ~1.0 m/s movement -> Walking.
4. ~2.5 m/s movement -> Running.
5. Flicker suppression via majority-vote smoothing.
6. Inactive track timeout and cleanup.
7. Explicit metric speeds (~0, ~1.0, ~2.5 m/s) with calibrated thresholds.
8. Transition boundary checks around 0.3 m/s and 1.75 m/s.
"""

import os
import sys

# Ensure yolo directory is in sys.path
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from activity_classifier import ActivityClassifier


def test_min_history_unknown():
    """Verify that fewer than min_samples returns Unknown."""
    clf = ActivityClassifier(window_sec=1.0, min_samples=3)
    act, spd = clf.update(track_id=1, world_pos=(0.0, 0.0), timestamp=0.0)
    assert act == "Unknown"
    act, spd = clf.update(track_id=1, world_pos=(0.0, 0.0), timestamp=0.1)
    assert act == "Unknown"


def test_stationary_is_staying():
    """Verify that a stationary track classifies as Staying."""
    clf = ActivityClassifier(
        window_sec=1.0, staying_thresh=0.3, running_thresh=1.75, smoothing_n=5
    )
    for i in range(15):
        t = i * 0.1
        act, spd = clf.update(track_id=1, world_pos=(2.0, 3.0), timestamp=t)

    assert spd < 0.01
    assert act == "Staying"


def test_walking_speed():
    """Verify that movement at ~1.0 m/s classifies as Walking."""
    clf = ActivityClassifier(
        window_sec=1.0, staying_thresh=0.3, running_thresh=1.75, smoothing_n=5
    )
    # At 10 Hz (dt = 0.1s), step 0.1m per frame = 1.0 m/s
    for i in range(15):
        t = i * 0.1
        x = i * 0.10
        act, spd = clf.update(track_id=1, world_pos=(x, 0.0), timestamp=t)

    assert 0.8 < spd < 1.2, f"Expected speed ~1.0, got {spd}"
    assert act == "Walking"


def test_running_speed():
    """Verify that movement at ~2.5 m/s classifies as Running."""
    clf = ActivityClassifier(
        window_sec=1.0, staying_thresh=0.3, running_thresh=1.75, smoothing_n=5
    )
    # At 10 Hz (dt = 0.1s), step 0.25m per frame = 2.5 m/s
    for i in range(15):
        t = i * 0.1
        x = i * 0.25
        act, spd = clf.update(track_id=1, world_pos=(x, 0.0), timestamp=t)

    assert spd > 1.75
    assert act == "Running"


def test_flicker_suppression():
    """Verify that an isolated anomalous frame does not cause state flicker."""
    clf = ActivityClassifier(
        window_sec=1.0, staying_thresh=0.3, running_thresh=1.75, smoothing_n=5
    )
    # Walk stably for 12 frames at 1.0 m/s
    x = 0.0
    for i in range(12):
        t = i * 0.1
        x = i * 0.10
        act, _ = clf.update(track_id=1, world_pos=(x, 0.0), timestamp=t)
    assert act == "Walking"

    # Inject 1 momentary standstill anomaly (speed 0 -> instantaneous Staying)
    t = 1.2
    act, _ = clf.update(track_id=1, world_pos=(x, 0.0), timestamp=t)
    assert act == "Walking", "Single anomalous frame should be suppressed by majority vote"

    # Next frame resumes walking
    t = 1.3
    x += 0.10
    act, _ = clf.update(track_id=1, world_pos=(x, 0.0), timestamp=t)
    assert act == "Walking"


def test_track_timeout():
    """Verify that inactive tracks are dropped after track_timeout_sec."""
    clf = ActivityClassifier(track_timeout_sec=1.5)
    pos = (1.0, 1.0)
    clf.update(track_id=1, world_pos=pos, timestamp=0.0)
    assert 1 in clf.tracks

    clf.update(track_id=2, world_pos=pos, timestamp=1.0)
    assert 1 in clf.tracks
    assert 2 in clf.tracks

    # Cleanup at t=1.6: Track 1 has elapsed 1.6s (> 1.5s), Track 2 has elapsed 0.6s
    dropped = clf.cleanup(current_timestamp=1.6)
    assert dropped == [1]
    assert 1 not in clf.tracks
    assert 2 in clf.tracks

    # Cleanup at t=2.6: Track 2 has elapsed 1.6s (> 1.5s)
    dropped = clf.cleanup(current_timestamp=2.6)
    assert dropped == [2]
    assert 2 not in clf.tracks


def test_metric_speeds_classification():
    """Verify exact ground-truth simulation speeds (~0, ~1.0, ~2.5 m/s)."""
    clf = ActivityClassifier(window_sec=1.0, staying_thresh=0.3, running_thresh=1.75)

    # Idle (0.0 m/s)
    for i in range(12):
        act_idle, spd_idle = clf.update(track_id=10, world_pos=(0.0, 0.0), timestamp=i * 0.1)
    assert act_idle == "Staying"
    assert spd_idle < 0.05

    # Walk (1.0 m/s)
    for i in range(12):
        act_walk, spd_walk = clf.update(track_id=20, world_pos=(i * 0.1, 0.0), timestamp=i * 0.1)
    assert act_walk == "Walking"
    assert abs(spd_walk - 1.0) < 0.1

    # Run (2.5 m/s)
    for i in range(12):
        act_run, spd_run = clf.update(track_id=30, world_pos=(i * 0.25, 0.0), timestamp=i * 0.1)
    assert act_run == "Running"
    assert abs(spd_run - 2.5) < 0.15


def test_threshold_boundaries():
    """Verify behavior around 0.3 m/s and 1.75 m/s boundaries."""
    clf = ActivityClassifier(window_sec=1.0, staying_thresh=0.3, running_thresh=1.75)

    # Just below 0.3 m/s (0.2 m/s) -> Staying
    for i in range(12):
        act, spd = clf.update(track_id=1, world_pos=(i * 0.02, 0.0), timestamp=i * 0.1)
    assert act == "Staying"

    # Just above 0.3 m/s (0.5 m/s) -> Walking
    for i in range(12):
        act, spd = clf.update(track_id=2, world_pos=(i * 0.05, 0.0), timestamp=i * 0.1)
    assert act == "Walking"

    # Just above 1.75 m/s (2.0 m/s) -> Running
    for i in range(12):
        act, spd = clf.update(track_id=3, world_pos=(i * 0.20, 0.0), timestamp=i * 0.1)
    assert act == "Running"
