"""Contract constants and native-resolution JPEG encoding for annotated camera feeds."""

import cv2
import numpy as np

OVERHEAD_COMPRESSED_TOPIC = "/yolo/annotated/compressed"
CORNER_COMPRESSED_TOPIC = "/yolo/corner/annotated/compressed"
OVERHEAD_RAW_TOPIC = "/yolo/annotated/raw"
CORNER_RAW_TOPIC = "/yolo/corner/annotated/raw"
JPEG_QUALITY = 75


def encode_native_jpeg(frame: np.ndarray, quality: int = JPEG_QUALITY) -> bytes:
    """Encode a frame without resizing; encoding errors are reported to the caller."""
    success, encoded = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not success:
        raise RuntimeError("OpenCV failed to encode an annotated frame as JPEG")
    return encoded.tobytes()
