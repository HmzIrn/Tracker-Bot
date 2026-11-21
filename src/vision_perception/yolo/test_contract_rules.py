"""Unit tests for contract-compliant activity severity and annotated stream sizing."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from annotated_stream import (
    CORNER_COMPRESSED_TOPIC,
    OVERHEAD_COMPRESSED_TOPIC,
    encode_native_jpeg,
)

def select_contract_activity(activities: list) -> str:
    """Select the single most serious activity according to contract: Staying > Running > Walking."""
    severity = {"Staying": 3, "Running": 2, "Walking": 1}
    valid = [a for a in activities if a in severity]
    if not valid:
        return ""
    return max(valid, key=lambda a: severity[a])


def test_severity_ranking():
    assert select_contract_activity(["Walking"]) == "Walking"
    assert select_contract_activity(["Walking", "Running"]) == "Running"
    assert select_contract_activity(["Running", "Walking"]) == "Running"
    assert select_contract_activity(["Walking", "Staying"]) == "Staying"
    assert select_contract_activity(["Running", "Staying"]) == "Staying"
    assert select_contract_activity(["Staying", "Running", "Walking"]) == "Staying"


def test_invalid_and_empty_publishes_nothing():
    assert select_contract_activity([]) == ""
    assert select_contract_activity(["Unknown"]) == ""
    assert select_contract_activity(["None"]) == ""
    assert select_contract_activity(["Unknown", "None"]) == ""


def test_annotated_stream_topics_and_native_dimensions():
    assert OVERHEAD_COMPRESSED_TOPIC == "/yolo/annotated/compressed"
    assert CORNER_COMPRESSED_TOPIC == "/yolo/corner/annotated/compressed"

    frame_overhead = np.zeros((480, 640, 3), dtype=np.uint8)
    frame_corner = np.zeros((480, 640, 3), dtype=np.uint8)
    import cv2
    for frame in (frame_overhead, frame_corner):
        decoded = cv2.imdecode(
            np.frombuffer(encode_native_jpeg(frame), dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        assert decoded.shape == (480, 640, 3)
