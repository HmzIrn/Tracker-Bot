#!/usr/bin/env python3
"""Multi-camera person tracking, ground projection fusion, activity classification,
and goal pose publisher node.

Subscribes:
  - /surveillance_camera/image_raw (sensor_msgs/Image, overhead camera, 30 Hz)
  - /corner_camera/image_raw       (sensor_msgs/Image, corner camera, 10 Hz)
  - /map                           (nav_msgs/OccupancyGrid, optional, transient-local)

Publishes:
  - /person_activity            (std_msgs/String, 'Staying' > 'Running' > 'Walking', 1-10 Hz)
  - /yolo/annotated/compressed  (sensor_msgs/CompressedImage, overhead | corner side-by-side, max 640x480, max 10 fps, quality ~70)
  - /goal_pose                  (geometry_msgs/PoseStamped, frame 'map', emitted once on staying >= 5s with >=15s cooldown)
  - /person_activity_json       (std_msgs/String, telemetry payload)
  - /camera/image_annotated     (sensor_msgs/Image, optional raw annotated frame)

Dependencies & Requirements:
  - numpy < 2.0 (Required to avoid C-ABI conflicts across PyTorch, OpenCV, and ROS 2 middleware)
  - Ultralytics YOLOv8, OpenCV (cv2), PyYAML
"""

import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

# Ensure local script directory (both link path and real path) are in sys.path
_script_dir = os.path.dirname(os.path.realpath(__file__))
for d in [os.path.dirname(os.path.abspath(__file__)), _script_dir]:
    if d and d not in sys.path:
        sys.path.insert(0, d)

# Ensure Python environments are searched in priority order: ~/yolo_env FIRST, then ~/yolo_venv
_env_candidates = [
    os.path.expanduser("~/yolo_venv"),
    os.path.expanduser("~/yolo_env"),
]
for _env_base in _env_candidates:
    for _subdir in [
        f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages",
        "lib/python3.12/site-packages",
    ]:
        _sp = os.path.join(_env_base, _subdir)
        if os.path.isdir(_sp):
            if _sp in sys.path:
                sys.path.remove(_sp)
            sys.path.insert(0, _sp)

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

from activity_classifier import ActivityClassifier
from goal_manager import GoalPoseManager, nudge_to_free_space, transform_world_to_map
from ground_projection import GroundProjector

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def load_cameras_config() -> Dict[str, dict]:
    """Load cameras.yaml via ament package share path."""
    candidates = []
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory("vision_perception")
            candidates.append(os.path.join(share, "config", "cameras.yaml"))
        except Exception:
            pass

    # Workspace fallback
    cur_dir = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.abspath(os.path.join(cur_dir, "..", "config", "cameras.yaml")))
    candidates.append(os.path.abspath(os.path.join(cur_dir, "..", "share", "vision_perception", "config", "cameras.yaml")))

    for path in candidates:
        if os.path.isfile(path):
            with open(path, "r") as f:
                return yaml.safe_load(f)

    # Hardcoded default fallback matching verified poses
    return {
        "overhead_camera": {
            "topic": "/surveillance_camera/image_raw",
            "pose": [0.0, 0.0, 12.2, 0.0, 1.5708, 1.5708],
            "width": 640,
            "height": 480,
            "hfov": 1.50,
            "ground_point": "center",
        },
        "corner_camera": {
            "topic": "/corner_camera/image_raw",
            "pose": [-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854],
            "width": 640,
            "height": 480,
            "hfov": 1.05,
            "ground_point": "bottom_center",
        },
    }


class YoloNode(Node):
    def __init__(self):
        # Force use_sim_time=True parameter override in constructor
        super().__init__(
            "yolo_node",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
        )

        # Load camera configuration from cameras.yaml
        self.cameras_cfg = load_cameras_config()
        overhead_cfg = self.cameras_cfg.get("overhead_camera", {})
        corner_cfg = self.cameras_cfg.get("corner_camera", {})

        overhead_topic_default = overhead_cfg.get("topic", "/surveillance_camera/image_raw")
        corner_topic_default = corner_cfg.get("topic", "/corner_camera/image_raw")

        # Declare ROS 2 parameters
        self.declare_parameter("overhead_topic", overhead_topic_default)
        self.declare_parameter("corner_topic", corner_topic_default)
        self.declare_parameter("output_compressed_topic", "/yolo/annotated/compressed")
        self.declare_parameter("output_raw_topic", "/camera/image_annotated")
        self.declare_parameter("activity_topic", "/person_activity")
        self.declare_parameter("activity_json_topic", "/person_activity_json")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("map_topic", "/map")

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
        self.declare_parameter("jpeg_quality", 70)  # Contract: quality about 70
        self.declare_parameter("publish_raw", False)  # Contract: default false
        self.declare_parameter("max_fps", 10.0)  # Contract: max 10 fps

        # Goal pose parameters (from cameras.yaml with ROS parameter overrides)
        cfg_goal = self.cameras_cfg.get("goal_pose", {})
        self.declare_parameter("staying_dwell_sec", float(cfg_goal.get("staying_dwell_sec", 5.0)))
        self.declare_parameter("goal_cooldown_sec", float(cfg_goal.get("goal_cooldown_sec", 15.0)))
        self.declare_parameter(
            "world_to_map", [float(v) for v in cfg_goal.get("world_to_map", [0.0, 0.0, 0.0])]
        )

        # Retrieve parameter values
        overhead_topic = str(self.get_parameter("overhead_topic").value)
        corner_topic = str(self.get_parameter("corner_topic").value)
        compressed_topic = str(self.get_parameter("output_compressed_topic").value)
        raw_topic = str(self.get_parameter("output_raw_topic").value)
        activity_topic = str(self.get_parameter("activity_topic").value)
        json_topic = str(self.get_parameter("activity_json_topic").value)
        goal_topic = str(self.get_parameter("goal_pose_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)

        self.conf = float(self.get_parameter("conf").value)
        self.device = str(self.get_parameter("device").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.fusion_dist_thresh = float(self.get_parameter("fusion_dist_thresh").value)
        self.tracking_dist_gate = float(self.get_parameter("tracking_dist_gate").value)
        self.track_timeout_sec = float(self.get_parameter("track_timeout_sec").value)

        staying_dwell_sec = float(self.get_parameter("staying_dwell_sec").value)
        goal_cooldown_sec = float(self.get_parameter("goal_cooldown_sec").value)
        world_to_map_val = [float(v) for v in self.get_parameter("world_to_map").value]

        # Initialize GroundProjector
        cam_dict = {k: v for k, v in self.cameras_cfg.items() if isinstance(v, dict) and "pose" in v}
        self.projector = GroundProjector(cam_dict)

        # Initialize Activity Classifier (metric m/s)
        self.classifier = ActivityClassifier(
            window_sec=float(self.get_parameter("window_sec").value),
            staying_thresh=float(self.get_parameter("staying_thresh").value),
            running_thresh=float(self.get_parameter("running_thresh").value),
            smoothing_n=int(self.get_parameter("smoothing_n").value),
            track_timeout_sec=self.track_timeout_sec,
        )

        # Initialize GoalPoseManager
        self.goal_mgr = GoalPoseManager(
            staying_dwell_sec=staying_dwell_sec,
            goal_cooldown_sec=goal_cooldown_sec,
            world_to_map=world_to_map_val,
        )

        # Map state and warning flag
        self.map_msg: Optional[OccupancyGrid] = None
        self.warned_no_map = False

        # Initialize YOLO model
        model_name = self.get_parameter("model").value
        self.get_logger().info(f"Loading YOLO model: {model_name} on device={self.device}...")
        self.model = YOLO(model_name)

        # Global tracks: global_id -> {pos: (x, y), last_seen: float, conf: float}
        self.global_tracks: Dict[int, dict] = {}
        self.next_global_id = 1

        # Buffers for cameras (overhead, corner)
        self.overhead_latest: Optional[Tuple[Image, np.ndarray, float]] = None
        self.corner_latest: Optional[Tuple[Image, np.ndarray, float]] = None
        self.last_infer_wall_time = 0.0

        # Activity publication rate tracking (Contract: 1 - 10 Hz while person visible)
        self.last_activity_pub_sim_time: float = -1.0
        self.last_activity_value: str = ""
        self.activity_heartbeat_timer = self.create_timer(1.0, self.on_activity_heartbeat)

        # Subscriptions (Using depth 10 to match parameter_bridge RELIABLE image publisher)
        self.sub_overhead = self.create_subscription(
            Image, overhead_topic, self.on_overhead_frame, 10
        )
        self.sub_corner = self.create_subscription(
            Image, corner_topic, self.on_corner_frame, 10
        )

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, map_topic, self.on_map, map_qos
        )

        # Publishers
        self.pub_activity = self.create_publisher(String, activity_topic, 10)
        self.pub_compressed = self.create_publisher(CompressedImage, compressed_topic, 10)
        self.pub_goal = self.create_publisher(PoseStamped, goal_topic, 10)
        self.pub_json = self.create_publisher(String, json_topic, 10)

        if self.publish_raw:
            self.pub_raw = self.create_publisher(Image, raw_topic, 10)
        else:
            self.pub_raw = None

        self.get_logger().info(
            f"YoloNode initialized successfully.\n"
            f"  Overhead camera: {overhead_topic}\n"
            f"  Corner camera:   {corner_topic}\n"
            f"  Activity topic:  {activity_topic}\n"
            f"  Compressed:      {compressed_topic} (max 640x480, max {self.max_fps} fps, quality {self.jpeg_quality})\n"
            f"  Goal pose:       {goal_topic} (dwell {staying_dwell_sec}s, cooldown {goal_cooldown_sec}s)"
        )

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_activity_heartbeat(self):
        """Ensure /person_activity is published at least 1 Hz while a person is visible."""
        if not self.last_activity_value:
            return
        now_sim = 0.0
        if self.overhead_latest is not None:
            now_sim = max(now_sim, self.overhead_latest[2])
        if self.corner_latest is not None:
            now_sim = max(now_sim, self.corner_latest[2])

        if now_sim > 0.0 and (now_sim - self.last_activity_pub_sim_time) >= 1.0:
            act_msg = String()
            act_msg.data = self.last_activity_value
            self.pub_activity.publish(act_msg)
            self.last_activity_pub_sim_time = now_sim

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

    def on_overhead_frame(self, msg: Image):
        try:
            bgr = self.to_bgr(msg)
            t = self.get_sim_time(msg)
            self.overhead_latest = (msg, bgr, t)
            self.try_step(t)
        except Exception as e:
            self.get_logger().error(f"Overhead camera error: {e}", throttle_duration_sec=5.0)

    def on_corner_frame(self, msg: Image):
        try:
            bgr = self.to_bgr(msg)
            t = self.get_sim_time(msg)
            self.corner_latest = (msg, bgr, t)
            self.try_step(t)
        except Exception as e:
            self.get_logger().error(f"Corner camera error: {e}", throttle_duration_sec=5.0)

    def try_step(self, sim_time: float):
        """Execute inference, fusion, and tracking step throttled by max_fps."""
        wall_now = time.monotonic()
        min_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.0

        if (wall_now - self.last_infer_wall_time) < (min_interval - 1e-4):
            return
        self.last_infer_wall_time = wall_now

        self.process_cameras(sim_time)

    def detect_camera(self, cam_id: str, bgr: np.ndarray) -> List[dict]:
        """Run YOLO person detection on a single camera frame and project to ground."""
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

                ground_pt = self.projector.project_bbox(cam_id, (x1, y1, x2, y2))
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
        # Retrieve latest frames
        frame_overhead = (
            self.overhead_latest[1]
            if self.overhead_latest is not None
            else np.zeros((480, 640, 3), dtype=np.uint8)
        )
        frame_corner = (
            self.corner_latest[1]
            if self.corner_latest is not None
            else np.zeros((480, 640, 3), dtype=np.uint8)
        )

        hdr_stamp = (
            self.overhead_latest[0].header.stamp
            if self.overhead_latest is not None
            else (self.corner_latest[0].header.stamp if self.corner_latest is not None else None)
        )

        dets_overhead = (
            self.detect_camera("overhead_camera", frame_overhead)
            if self.overhead_latest is not None
            else []
        )
        dets_corner = (
            self.detect_camera("corner_camera", frame_corner)
            if self.corner_latest is not None
            else []
        )

        # Multi-camera fusion: merge detections within fusion_dist_thresh (~0.8m)
        fused_items: List[dict] = []
        matched_corner_indices = set()

        for d_over in dets_overhead:
            p_over = d_over["ground_pos"]
            best_match_idx = None
            best_dist = self.fusion_dist_thresh

            for idx_c, d_corn in enumerate(dets_corner):
                if idx_c in matched_corner_indices:
                    continue
                p_corn = d_corn["ground_pos"]
                dist = math.hypot(p_over[0] - p_corn[0], p_over[1] - p_corn[1])
                if dist < best_dist:
                    best_dist = dist
                    best_match_idx = idx_c

            if best_match_idx is not None:
                d_corn = dets_corner[best_match_idx]
                matched_corner_indices.add(best_match_idx)
                w1, w2 = d_over["conf"], d_corn["conf"]
                total_w = w1 + w2
                fx = (d_over["ground_pos"][0] * w1 + d_corn["ground_pos"][0] * w2) / total_w
                fy = (d_over["ground_pos"][1] * w1 + d_corn["ground_pos"][1] * w2) / total_w
                fused_items.append({
                    "pos": (fx, fy),
                    "conf": max(w1, w2),
                    "cameras_seen": ["overhead_camera", "corner_camera"],
                    "overhead_box": d_over["bbox"],
                    "corner_box": d_corn["bbox"],
                })
            else:
                fused_items.append({
                    "pos": d_over["ground_pos"],
                    "conf": d_over["conf"],
                    "cameras_seen": ["overhead_camera"],
                    "overhead_box": d_over["bbox"],
                    "corner_box": None,
                })

        for idx_c, d_corn in enumerate(dets_corner):
            if idx_c not in matched_corner_indices:
                fused_items.append({
                    "pos": d_corn["ground_pos"],
                    "conf": d_corn["conf"],
                    "cameras_seen": ["corner_camera"],
                    "overhead_box": None,
                    "corner_box": d_corn["bbox"],
                })

        # Nearest-neighbor tracking association to previous global tracks
        assigned_tracks: List[dict] = []
        unmatched_fused = list(range(len(fused_items)))
        unmatched_tids = list(self.global_tracks.keys())

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
                "overhead_box": item["overhead_box"],
                "corner_box": item["corner_box"],
            })

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
                    "overhead_box": item["overhead_box"],
                    "corner_box": item["corner_box"],
                })

        # Cleanup lost tracks
        dropped = self.classifier.cleanup(current_time)
        for dtid in dropped:
            if dtid in self.global_tracks:
                del self.global_tracks[dtid]

        for tid, tinfo in list(self.global_tracks.items()):
            if current_time - tinfo["last_seen"] > self.track_timeout_sec:
                del self.global_tracks[tid]

        self.goal_mgr.cleanup(set(self.global_tracks.keys()))

        # Update ActivityClassifier & GoalPoseManager for all assigned tracks
        person_entries = []
        valid_activities: List[str] = []

        # Contract priority order: Staying > Running > Walking
        severity_rank = {"Staying": 3, "Running": 2, "Walking": 1}

        world_to_map = [float(v) for v in self.get_parameter("world_to_map").value]

        for trk in assigned_tracks:
            gid = trk["global_id"]
            pos = trk["pos"]
            conf = trk["conf"]
            act, spd = self.classifier.update(gid, pos, current_time)
            trk["activity"] = act
            trk["speed"] = spd

            # Convert to map frame coordinates for display and telemetry
            map_x, map_y = transform_world_to_map(pos[0], pos[1], world_to_map)
            trk["map_pos"] = (map_x, map_y)

            if act in severity_rank:
                valid_activities.append(act)

            # Check goal pose emission trigger
            goal_cand = self.goal_mgr.update_track(gid, act, pos, current_time)
            if goal_cand is not None:
                self.handle_goal_pose_emission(gid, goal_cand, hdr_stamp, current_time)

            bbox = trk["overhead_box"] if trk["overhead_box"] is not None else trk["corner_box"]

            person_entries.append({
                "global_id": gid,
                "track_id": gid,
                "activity": act,
                "speed_mps": round(spd, 3),
                "x_world": round(pos[0], 3),
                "y_world": round(pos[1], 3),
                "x_map": round(map_x, 3),
                "y_map": round(map_y, 3),
                "cameras_seen": trk["cameras_seen"],
                "confidence": round(conf, 3),
                "bbox": [round(float(v), 1) for v in bbox] if bbox is not None else None,
            })

        # Contract: /person_activity
        # Exactly "Walking", "Running", or "Staying". NEVER "None" or "Unknown".
        # Publish NOTHING when nobody is classified.
        # Most serious label: Staying > Running > Walking.
        # Rate: at least 1 Hz and at most 10 Hz while a person is visible.
        if valid_activities:
            most_serious = max(valid_activities, key=lambda a: severity_rank.get(a, 0))
            self.last_activity_value = most_serious
            min_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.1
            if (current_time - self.last_activity_pub_sim_time) >= (min_interval - 1e-4):
                act_msg = String()
                act_msg.data = most_serious
                self.pub_activity.publish(act_msg)
                self.last_activity_pub_sim_time = current_time
        else:
            self.last_activity_value = ""

        # Telemetry topic /person_activity_json
        json_msg = String()
        json_msg.data = json.dumps({
            "timestamp": round(current_time, 3),
            "entries": person_entries,
        })
        self.pub_json.publish(json_msg)

        # Annotated stream: side-by-side overhead + corner, each resized to 320x240 (total 640x240)
        self.publish_annotated_stream(
            frame_overhead, frame_corner, assigned_tracks, hdr_stamp
        )

    def handle_goal_pose_emission(
        self,
        track_id: int,
        map_coords: Tuple[float, float],
        hdr_stamp,
        current_time: float,
    ):
        """Perform free-space verification and publish goal pose."""
        tx, ty = map_coords

        if self.map_msg is not None:
            grid = self.map_msg.data
            w = self.map_msg.info.width
            h = self.map_msg.info.height
            res = self.map_msg.info.resolution
            ox = self.map_msg.info.origin.position.x
            oy = self.map_msg.info.origin.position.y

            nudged = nudge_to_free_space(
                tx, ty, grid, w, h, res, ox, oy, max_radius=0.5
            )
            if nudged is None:
                self.get_logger().warn(
                    f"Goal pose for track {track_id} at map ({tx:.2f}, {ty:.2f}) "
                    f"is not in free space and no free cell found within 0.5m. Skipping publish."
                )
                return
            final_x, final_y = nudged
        else:
            if not self.warned_no_map:
                self.get_logger().warn("No /map received yet; publishing unadjusted /goal_pose.")
                self.warned_no_map = True
            final_x, final_y = tx, ty

        # Publish PoseStamped
        goal_msg = PoseStamped()
        if hdr_stamp is not None:
            goal_msg.header.stamp = hdr_stamp
        else:
            goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = float(final_x)
        goal_msg.pose.position.y = float(final_y)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.pub_goal.publish(goal_msg)
        self.goal_mgr.mark_fired(track_id, current_time)
        self.get_logger().info(
            f"Published /goal_pose for track {track_id} at map: ({final_x:.2f}, {final_y:.2f})"
        )

    def publish_annotated_stream(
        self,
        frame_overhead: np.ndarray,
        frame_corner: np.ndarray,
        assigned_tracks: List[dict],
        hdr_stamp,
    ):
        """Compose side-by-side stream (overhead first, corner second) resized to 320x240 each."""
        # Visual color palette
        color_map = {
            "Walking": (0, 200, 0),
            "Running": (0, 140, 255),
            "Staying": (0, 0, 220),
            "Unknown": (128, 128, 128),
        }

        # Annotate full-resolution frames before resizing
        annotated_over = frame_overhead.copy()
        annotated_corn = frame_corner.copy()

        for trk in assigned_tracks:
            gid = trk["global_id"]
            act = trk["activity"]
            color = color_map.get(act, (200, 200, 200))
            mx, my = trk.get("map_pos", (0.0, 0.0))
            label = f"ID {gid}: {act} [{mx:.1f},{my:.1f}m]"

            # Draw on overhead camera
            if trk["overhead_box"] is not None:
                x1, y1, x2, y2 = [int(v) for v in trk["overhead_box"]]
                cv2.rectangle(annotated_over, (x1, y1), (x2, y2), color, 2)
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated_over, (x1, max(0, y1 - lh - 6)), (x1 + lw + 6, y1), color, -1)
                cv2.putText(
                    annotated_over, label, (x1 + 3, max(lh + 2, y1 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
                )

            # Draw on corner camera
            if trk["corner_box"] is not None:
                x1, y1, x2, y2 = [int(v) for v in trk["corner_box"]]
                cv2.rectangle(annotated_corn, (x1, y1), (x2, y2), color, 2)
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated_corn, (x1, max(0, y1 - lh - 6)), (x1 + lw + 6, y1), color, -1)
                cv2.putText(
                    annotated_corn, label, (x1 + 3, max(lh + 2, y1 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
                )

        # Header titles
        cv2.putText(annotated_over, "Overhead (Top-Down)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(annotated_corn, "Corner Mount", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Resize each to 320x240 so combined image is 640x240 (at most 640x480)
        over_small = cv2.resize(annotated_over, (320, 240), interpolation=cv2.INTER_AREA)
        corn_small = cv2.resize(annotated_corn, (320, 240), interpolation=cv2.INTER_AREA)

        # Overhead view first, Corner view second
        combined = np.hstack([over_small, corn_small])

        # JPEG compression
        success, enc = cv2.imencode(
            ".jpg", combined, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if success:
            comp_msg = CompressedImage()
            if hdr_stamp is not None:
                comp_msg.header.stamp = hdr_stamp
            comp_msg.header.frame_id = "overhead_camera_link"
            comp_msg.format = "jpeg"
            comp_msg.data = enc.tobytes()
            self.pub_compressed.publish(comp_msg)

        if self.publish_raw and self.pub_raw is not None:
            raw_msg = Image()
            if hdr_stamp is not None:
                raw_msg.header.stamp = hdr_stamp
            raw_msg.header.frame_id = "overhead_camera_link"
            raw_msg.height, raw_msg.width = combined.shape[:2]
            raw_msg.encoding = "bgr8"
            raw_msg.is_bigendian = 0
            raw_msg.step = int(combined.shape[1] * 3)
            raw_msg.data = combined.tobytes()
            self.pub_raw.publish(raw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
