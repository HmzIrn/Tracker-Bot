import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from activity_publisher import ActivityPublisherState


def test_label_changes_publish_immediately_with_ten_hz_floor():
    publisher = ActivityPublisherState(2.0)

    assert publisher.update("Walking", 0.0) == "Walking"
    assert publisher.update("Running", 0.05) is None
    assert publisher.heartbeat(0.099) is None
    assert publisher.heartbeat(0.1) == "Running"
    assert publisher.heartbeat(0.2) is None


def test_repeats_at_configured_wall_clock_rate():
    publisher = ActivityPublisherState(2.0)

    assert publisher.update("Staying", 5.0) == "Staying"
    assert publisher.heartbeat(5.499) is None
    assert publisher.heartbeat(5.5) == "Staying"


def test_publish_rate_is_clamped_and_non_finite_values_are_rejected():
    slow = ActivityPublisherState(0.5)
    fast = ActivityPublisherState(20.0)

    assert slow.activity_publish_hz == 1.5
    assert slow.repeat_interval_sec == 0.6
    assert fast.activity_publish_hz == 10.0
    assert fast.repeat_interval_sec == 0.1
    with pytest.raises(ValueError, match="must be finite"):
        ActivityPublisherState(float("nan"))


def test_unknown_or_missing_activity_stays_silent():
    publisher = ActivityPublisherState()

    assert publisher.update("", 0.0) is None
    assert publisher.update("Unknown", 0.5) is None
    assert publisher.heartbeat(1.0) is None
    assert publisher.update("Walking", 1.5) == "Walking"
    assert publisher.update("Unknown", 1.6) is None
    assert publisher.heartbeat(2.5) is None


def test_repeating_stops_at_three_seconds_without_inference():
    publisher = ActivityPublisherState()

    assert publisher.update("Staying", 10.0) == "Staying"
    assert publisher.heartbeat(12.99) == "Staying"
    assert publisher.heartbeat(13.0) is None
    assert publisher.heartbeat(13.5) is None
    assert publisher.update("Staying", 13.6) == "Staying"
