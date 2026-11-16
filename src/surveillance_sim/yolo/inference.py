#!/usr/bin/env python3
"""Multi-camera person tracking, ground projection fusion, and activity classification node.

Subscribes:
  - /camera/image_raw   (sensor_msgs/Image, camera 1)
  - /camera2/image_raw  (sensor_msgs/Image, camera 2)

Publishes:
  - /person_activity           (std_msgs/String, prominent person activity or 'None')
  - /person_activity_json      (std_msgs/String, JSON telemetry with global_id, x, y, speed_mps, cameras_seen)
  - /yolo/annotated/compressed (sensor_msgs/CompressedImage, cam1 | cam2 side-by-side JPEG)
  - /camera/image_annotated    (sensor_msgs/Image, raw annotated frame, gated by publish_raw)

Dependencies:
  - Pure Python + numpy + cv2 + ultralytics + ground_projection + activity_classifier
"""

import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

# Ensure local module directory is searchable
_script_dir = os.path.dirname(os.path.realpath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

# Ensure ~/yolo_venv site-packages is available if not in active venv
try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
except ImportError:
    _venv_site = os.path.expanduser("~/yolo_venv/lib/python3.12/site-packages")
    if os.path.exists(_venv_site) and _venv_site not in sys.path:
        sys.path.insert(0, _venv_site)
    import cv2
    import numpy as np
    from ultralytics import YOLO

from activity_classifier import ActivityClassifier
from ground_projection import GroundProjector, project_pixel_to_ground

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

# Default camera calibration
DEFAULT_CAMERAS = {
    "camera1": {
        "pose": [-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854],
        "width": 640,
        "height": 480,
        "hfov": 1.05,
    },
    "camera2": {
        "pose": [7.0, 7.0, 2.3, 0.0, 0.25, -2.356],
        "width": 640,
        "height": 480,
        "hfov": 1.05,
    },
}


class MultiCameraTrackerNode(Node):
    def __init__(self):
        super().__init__("yolo_inference")

        # Declare ROS 2 parameters
        self.declare_parameter("cam1_topic", "/camera/image_raw")
        self.declare_parameter("cam2_topic", "/camera2/image_raw")
        self.declare_parameter("output_compressed_topic", "/yolo/annotated/compressed")
        self.declare_parameter("output_raw_topic", "/camera/image_annotated")
        self.declare_parameter("activity_topic", "/person_activity")
        self.declare_parameter("activity_json_topic", "/person_activity_json")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.15)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("window_sec", 1.0)
        self.declare_parameter("staying_thresh", 0.3)
        self.declare_parameter("running_thresh", 1.75)
        self.declare_parameter("smoothing_n", 5)
        self.declare_parameter("track_timeout_sec", 2.0)
        self.declare_parameter("fusion_dist_thresh", 0.8)
        self.declare_parameter("tracking_dist_gate", 1.5)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("max_fps", 15.0)

        # Retrieve parameter values
        cam1_topic = self.get_parameter("cam1_topic").value
        cam2_topic = self.get_parameter("cam2_topic").value
        compressed_topic = self.get_parameter("output_compressed_topic").value
        raw_topic = self.get_parameter("output_raw_topic").value
        activity_topic = self.get_parameter("activity_topic").value
        json_topic = self.get_parameter("activity_json_topic").value

        self.conf = float(self.get_parameter("conf").value)
        self.device = str(self.get_parameter("device").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.fusion_dist_thresh = float(self.get_parameter("fusion_dist_thresh").value)
        self.tracking_dist_gate = float(self.get_parameter("tracking_dist_gate").value)
        self.track_timeout_sec = float(self.get_parameter("track_timeout_sec").value)

        # Load camera config
        self.projector = GroundProjector(DEFAULT_CAMERAS)

        # Initialize Activity Classifier (metric m/s)
        self.classifier = ActivityClassifier(
            window_sec=float(self.get_parameter("window_sec").value),
            staying_thresh=float(self.get_parameter("staying_thresh").value),
            running_thresh=float(self.get_parameter("running_thresh").value),
            smoothing_n=int(self.get_parameter("smoothing_n").value),
            track_timeout_sec=self.track_timeout_sec,
        )

        # Initialize YOLO model
        model_name = self.get_parameter("model").value
        self.get_logger().info(f"Loading YOLO model: {model_name} on device={self.device}...")
        self.model = YOLO(model_name)

        # Global tracks: global_id -> {pos: (x, y), last_seen: float, conf: float}
        self.global_tracks: Dict[int, dict] = {}
        self.next_global_id = 1

        # Buffers for dual cameras
        self.cam1_latest: Optional[Tuple[Image, np.ndarray, float]] = None
        self.cam2_latest: Optional[Tuple[Image, np.ndarray, float]] = None
        self.last_infer_sim_time = -1.0
        self.last_infer_wall_time = 0.0

        # Subscriptions
        self.sub_cam1 = self.create_subscription(
            Image, cam1_topic, self.on_cam1, qos_profile_sensor_data
        )
        self.sub_cam2 = self.create_subscription(
            Image, cam2_topic, self.on_cam2, qos_profile_sensor_data
        )

        # Publishers
        self.pub_compressed = self.create_publisher(CompressedImage, compressed_topic, 10)
        self.pub_activity = self.create_publisher(String, activity_topic, 10)
        self.pub_json = self.create_publisher(String, json_topic, 10)

        if self.publish_raw:
            self.pub_raw = self.create_publisher(Image, raw_topic, 10)
        else:
            self.pub_raw = None

        self.get_logger().info(
            f"MultiCameraTrackerNode started.\n"
            f"  Subscribed to: {cam1_topic}, {cam2_topic}\n"
            f"  Publishing: {compressed_topic}, {activity_topic}, {json_topic}"
            + (f", {raw_topic}" if self.publish_raw else "")
        )

    @staticmethod
    def to_bgr(msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to BGR numpy array without cv_bridge."""
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            frame = frame[:, :, ::-1]
        elif msg.encoding != "bgr8":
            raise ValueError(f"Unsupported encoding: {msg.encoding}")
        return np.ascontiguousarray(frame)

    @staticmethod
    def get_sim_time(msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def on_cam1(self, msg: Image):
        try:
            bgr = self.to_bgr(msg)
            t = self.get_sim_time(msg)
            self.cam1_latest = (msg, bgr, t)
            self.try_step(t)
        except Exception as e:
            self.get_logger().error(f"Cam1 error: {e}", throttle_duration_sec=5.0)

    def on_cam2(self, msg: Image):
        try:
            bgr = self.to_bgr(msg)
            t = self.get_sim_time(msg)
            self.cam2_latest = (msg, bgr, t)
            self.try_step(t)
        except Exception as e:
            self.get_logger().error(f"Cam2 error: {e}", throttle_duration_sec=5.0)

    def try_step(self, sim_time: float):
        """Execute tracking & fusion step throttled by max_fps."""
        wall_now = time.monotonic()
        min_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.0

        if sim_time > 0.0:
            if self.last_infer_sim_time >= 0.0 and (sim_time - self.last_infer_sim_time) < (min_interval - 1e-4):
                return
            self.last_infer_sim_time = sim_time
        else:
            if (wall_now - self.last_infer_wall_time) < (min_interval - 1e-4):
                return
            self.last_infer_wall_time = wall_now

        current_time = sim_time if sim_time > 0.0 else wall_now
        self.process_cameras(current_time)

    def detect_camera(
        self, cam_id: str, bgr: np.ndarray
    ) -> List[dict]:
        """Run YOLO person detection on a single camera frame and project to floor."""
        results = self.model(
            bgr,
            classes=[0],
            conf=self.conf,
            device=self.device,
            verbose=False,
        )
        r = results[0]
        detections = []
        if r.boxes is not None and len(r.boxes) > 0:
            xyxy = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            for i in range(len(xyxy)):
                box = xyxy[i]
                x1, y1, x2, y2 = [float(v) for v in box]
                conf = float(confs[i])
                u_feet = (x1 + x2) / 2.0
                v_feet = y2
                ground_pt = self.projector.project(cam_id, u_feet, v_feet)
                if ground_pt is not None:
                    gx, gy = ground_pt
                    detections.append({
                        "cam_id": cam_id,
                        "bbox": (x1, y1, x2, y2),
                        "conf": conf,
                        "ground_pos": (gx, gy),
                    })
        return detections

    def process_cameras(self, current_time: float):
        # Retrieve frames
        frame1 = self.cam1_latest[1] if self.cam1_latest is not None else np.zeros((480, 640, 3), dtype=np.uint8)
        frame2 = self.cam2_latest[1] if self.cam2_latest is not None else np.zeros((480, 640, 3), dtype=np.uint8)
        hdr_stamp = self.cam1_latest[0].header.stamp if self.cam1_latest is not None else (
            self.cam2_latest[0].header.stamp if self.cam2_latest is not None else None
        )

        dets1 = self.detect_camera("camera1", frame1) if self.cam1_latest is not None else []
        dets2 = self.detect_camera("camera2", frame2) if self.cam2_latest is not None else []

        # Simple fusion: merge detections within fusion_dist_thresh (~0.8m)
        fused_items: List[dict] = []
        matched_cam2_indices = set()

        for d1 in dets1:
            p1 = d1["ground_pos"]
            best_match_idx = None
            best_dist = self.fusion_dist_thresh

            for idx2, d2 in enumerate(dets2):
                if idx2 in matched_cam2_indices:
                    continue
                p2 = d2["ground_pos"]
                dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                if dist < best_dist:
                    best_dist = dist
                    best_match_idx = idx2

            if best_match_idx is not None:
                d2 = dets2[best_match_idx]
                matched_cam2_indices.add(best_match_idx)
                # Weighted average position
                w1, w2 = d1["conf"], d2["conf"]
                total_w = w1 + w2
                fx = (d1["ground_pos"][0] * w1 + d2["ground_pos"][0] * w2) / total_w
                fy = (d1["ground_pos"][1] * w1 + d2["ground_pos"][1] * w2) / total_w
                fused_items.append({
                    "pos": (fx, fy),
                    "conf": max(w1, w2),
                    "cameras_seen": ["camera1", "camera2"],
                    "cam1_box": d1["bbox"],
                    "cam2_box": d2["bbox"],
                })
            else:
                fused_items.append({
                    "pos": d1["ground_pos"],
                    "conf": d1["conf"],
                    "cameras_seen": ["camera1"],
                    "cam1_box": d1["bbox"],
                    "cam2_box": None,
                })

        for idx2, d2 in enumerate(dets2):
            if idx2 not in matched_cam2_indices:
                fused_items.append({
                    "pos": d2["ground_pos"],
                    "conf": d2["conf"],
                    "cameras_seen": ["camera2"],
                    "cam1_box": None,
                    "cam2_box": d2["bbox"],
                })

        # Nearest-neighbor matching to previous global tracks
        assigned_tracks: List[dict] = []
        unmatched_fused = list(range(len(fused_items)))
        unmatched_tids = list(self.global_tracks.keys())

        # Pairwise distances
        pairs = []
        for fi in unmatched_fused:
            fx, fy = fused_items[fi]["pos"]
            for tid in unmatched_tids:
                tx, ty = self.global_tracks[tid]["pos"]
                d = math.hypot(fx - tx, fy - ty)
                if d <= self.tracking_dist_gate:
                    pairs.append((d, fi, tid))

        pairs.sort(key=lambda x: x[0])
        used_fi = set()
        used_tid = set()

        for d, fi, tid in pairs:
            if fi in used_fi or tid in used_tid:
                continue
            used_fi.add(fi)
            used_tid.add(tid)
            item = fused_items[fi]
            self.global_tracks[tid]["pos"] = item["pos"]
            self.global_tracks[tid]["last_seen"] = current_time
            self.global_tracks[tid]["conf"] = item["conf"]
            assigned_tracks.append({
                "global_id": tid,
                "pos": item["pos"],
                "conf": item["conf"],
                "cameras_seen": item["cameras_seen"],
                "cam1_box": item["cam1_box"],
                "cam2_box": item["cam2_box"],
            })

        # Unmatched fused items get new global IDs
        for fi in range(len(fused_items)):
            if fi not in used_fi:
                item = fused_items[fi]
                tid = self.next_global_id
                self.next_global_id += 1
                self.global_tracks[tid] = {
                    "pos": item["pos"],
                    "last_seen": current_time,
                    "conf": item["conf"],
                }
                assigned_tracks.append({
                    "global_id": tid,
                    "pos": item["pos"],
                    "conf": item["conf"],
                    "cameras_seen": item["cameras_seen"],
                    "cam1_box": item["cam1_box"],
                    "cam2_box": item["cam2_box"],
                })

        # Drop old tracks
        dropped = self.classifier.cleanup(current_time)
        for dtid in dropped:
            if dtid in self.global_tracks:
                del self.global_tracks[dtid]

        # Also cleanup global_tracks not in classifier
        for tid, tinfo in list(self.global_tracks.items()):
            if current_time - tinfo["last_seen"] > self.track_timeout_sec:
                del self.global_tracks[tid]

        # Update ActivityClassifier for all assigned tracks
        person_entries = []
        prominent_activity = "None"
        prominent_conf = -1.0

        for trk in assigned_tracks:
            gid = trk["global_id"]
            pos = trk["pos"]
            conf = trk["conf"]
            act, spd = self.classifier.update(gid, pos, current_time)
            trk["activity"] = act
            trk["speed"] = spd

            # Select prominent person by confidence
            if conf > prominent_conf and act != "None":
                prominent_conf = conf
                prominent_activity = act

            # Determine dominant bbox for compatibility
            bbox = trk["cam1_box"] if trk["cam1_box"] is not None else trk["cam2_box"]

            person_entries.append({
                "global_id": gid,
                "track_id": gid,
                "activity": act,
                "speed": round(spd, 3),
                "speed_mps": round(spd, 3),
                "x": round(pos[0], 3),
                "y": round(pos[1], 3),
                "cameras_seen": trk["cameras_seen"],
                "confidence": round(conf, 3),
                "bbox": [round(float(v), 1) for v in bbox] if bbox is not None else None,
            })

        # 1. Publish /person_activity
        act_msg = String()
        act_msg.data = prominent_activity
        self.pub_activity.publish(act_msg)

        # 2. Publish /person_activity_json
        json_msg = String()
        json_msg.data = json.dumps({
            "timestamp": round(current_time, 3),
            "entries": person_entries,
        })
        self.pub_json.publish(json_msg)

        # 3. Annotate frames and publish side-by-side /yolo/annotated/compressed
        annotated_1 = frame1.copy()
        annotated_2 = frame2.copy()

        # Visual color palette: Walking=Green, Running=Orange, Staying=Red, Unknown=Gray
        color_map = {
            "Walking": (0, 200, 0),
            "Running": (0, 140, 255),
            "Staying": (0, 0, 220),
            "Unknown": (128, 128, 128),
        }

        for trk in assigned_tracks:
            gid = trk["global_id"]
            act = trk["activity"]
            color = color_map.get(act, (200, 200, 200))
            label = f"G{gid}: {act}"

            # Draw on Camera 1 if visible
            if trk["cam1_box"] is not None:
                x1, y1, x2, y2 = [int(v) for v in trk["cam1_box"]]
                cv2.rectangle(annotated_1, (x1, y1), (x2, y2), color, 2)
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated_1, (x1, max(0, y1 - lh - 6)), (x1 + lw + 6, y1), color, -1)
                cv2.putText(
                    annotated_1, label, (x1 + 3, max(lh + 2, y1 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
                )

            # Draw on Camera 2 if visible
            if trk["cam2_box"] is not None:
                x1, y1, x2, y2 = [int(v) for v in trk["cam2_box"]]
                cv2.rectangle(annotated_2, (x1, y1), (x2, y2), color, 2)
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated_2, (x1, max(0, y1 - lh - 6)), (x1 + lw + 6, y1), color, -1)
                cv2.putText(
                    annotated_2, label, (x1 + 3, max(lh + 2, y1 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
                )

        # Header titles on images
        cv2.putText(annotated_1, "Camera 1 (SW)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(annotated_2, "Camera 2 (NE)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Side-by-side combined image: 1280x480
        combined = np.hstack([annotated_1, annotated_2])

        # Encode JPEG
        success, enc = cv2.imencode(
            ".jpg", combined, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if success:
            comp_msg = CompressedImage()
            if hdr_stamp is not None:
                comp_msg.header.stamp = hdr_stamp
            comp_msg.header.frame_id = "camera_optical_link"
            comp_msg.format = "jpeg"
            comp_msg.data = enc.tobytes()
            self.pub_compressed.publish(comp_msg)

        # 4. Publish /camera/image_annotated (raw) if enabled
        if self.publish_raw and self.pub_raw is not None:
            raw_msg = Image()
            if hdr_stamp is not None:
                raw_msg.header.stamp = hdr_stamp
            raw_msg.header.frame_id = "camera_optical_link"
            raw_msg.height, raw_msg.width = combined.shape[:2]
            raw_msg.encoding = "bgr8"
            raw_msg.is_bigendian = 0
            raw_msg.step = int(combined.shape[1] * 3)
            raw_msg.data = combined.tobytes()
            self.pub_raw.publish(raw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
