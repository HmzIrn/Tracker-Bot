"""Unit tests for contract-compliant activity severity and annotated stream sizing."""

import numpy as np
import pytest


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


def test_annotated_stream_dimensions():
    # Two 640x480 source frames
    frame_overhead = np.zeros((480, 640, 3), dtype=np.uint8)
    frame_corner = np.zeros((480, 640, 3), dtype=np.uint8)

    # Resized to 320x240 each
    import cv2
    over_small = cv2.resize(frame_overhead, (320, 240))
    corn_small = cv2.resize(frame_corner, (320, 240))

    # Composed side-by-side (overhead first)
    combined = np.hstack([over_small, corn_small])

    height, width, channels = combined.shape
    assert height == 240
    assert width == 640
    assert width <= 640
    assert height <= 480
    assert channels == 3
