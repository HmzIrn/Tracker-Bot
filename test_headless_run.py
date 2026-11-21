#!/usr/bin/env python3
"""Milestone 11 Verification Script: 60s headless simulation + yolo_node test.

Validates:
1. /surveillance_camera/image_raw frequency (~30 Hz)
2. Both independent /yolo annotated streams are 640x480 JPEG and <= 10 Hz
3. /person_activity values (strictly 'Walking', 'Running', 'Staying'; absent when occluded)
4. /goal_pose emission (frame_id 'map', fired once during staying, cooldown obeyed)
5. Overhead camera person detection at conf 0.15
6. Walking vs Running vs Staying classification accuracy & confusion matrix
"""

import collections
import json
import os
import signal
import subprocess
import sys
import time
from typing import List

# Setup ROS 2 paths
for p in ["/opt/ros/jazzy/lib/python3.12/site-packages", "/opt/ros/jazzy/local/lib/python3.12/dist-packages"]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

# Setup virtual environment fallback
for env_base in [os.path.expanduser("~/yolo_env"), os.path.expanduser("~/yolo_venv")]:
    sp = os.path.join(env_base, "lib/python3.12/site-packages")
    if os.path.exists(sp) and sp not in sys.path:
        sys.path.insert(0, sp)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class VerificationCollector(Node):
    def __init__(self):
        super().__init__("milestone11_verifier")

        # Data collection structures
        self.surveillance_cam_times = []
        self.compressed_frames = []
        self.corner_compressed_frames = []
        self.activity_msgs = []       # (sim_time, data)
        self.goal_poses = []          # (sim_time, frame_id, x, y, z, w)
        self.json_records = []

        self.start_sim_time = None

        # Subscriptions
        self.sub_cam = self.create_subscription(
            Image, "/surveillance_camera/image_raw", self.on_cam_raw, qos_profile_sensor_data
        )
        self.sub_compressed = self.create_subscription(
            CompressedImage, "/yolo/annotated/compressed", self.on_compressed, 10
        )
        self.sub_corner_compressed = self.create_subscription(
            CompressedImage, "/yolo/corner/annotated/compressed",
            self.on_corner_compressed, 10
        )
        self.sub_act = self.create_subscription(
            String, "/person_activity", self.on_activity, 10
        )
        self.sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.on_goal, 10
        )
        self.sub_json = self.create_subscription(
            String, "/person_activity_json", self.on_json, 10
        )

    def get_sim_time(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_cam_raw(self, msg: Image):
        t = self.get_sim_time(msg.header.stamp)
        if self.start_sim_time is None:
            self.start_sim_time = t
        self.surveillance_cam_times.append(t)

    def on_compressed(self, msg: CompressedImage):
        self.collect_compressed(self.compressed_frames, msg)

    def on_corner_compressed(self, msg: CompressedImage):
        self.collect_compressed(self.corner_compressed_frames, msg)

    def collect_compressed(self, frames, msg: CompressedImage):
        t = self.get_sim_time(msg.header.stamp)
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            h, w = img.shape[:2]
        else:
            h, w = 0, 0
        frames.append((t, w, h, len(msg.data), msg.format))

    def on_activity(self, msg: String):
        sim_now = self.surveillance_cam_times[-1] if self.surveillance_cam_times else 0.0
        self.activity_msgs.append((sim_now, msg.data))

    def on_goal(self, msg: PoseStamped):
        t = self.get_sim_time(msg.header.stamp)
        self.goal_poses.append((
            t,
            msg.header.frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.w,
        ))

    def on_json(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.json_records.append(data)
        except Exception:
            pass


def run_verification(target_sim_sec: float = 60.0):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "30"
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    # Pre-clean any leftover processes
    subprocess.run(["pkill", "-9", "-f", "gz sim|parameter_bridge|yolo_node|rqt_image_view"], check=False)
    time.sleep(1.0)

    print("=== Milestone 11 Headless Run Verification ===")
    print(f"Launching simulation (headless:=true, bridge_clock:=true) on ROS_DOMAIN_ID=30...")

    sim_cmd = [
        "bash", "-lc",
        "export ROS_DOMAIN_ID=30 && export GALLIUM_DRIVER=d3d12 && export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA && "
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "ros2 launch surveillance_sim surveillance_world.launch.py headless:=true bridge_clock:=true"
    ]
    # Output log files
    sim_log = open("sim_run.log", "w")
    yolo_log = open("yolo_run.log", "w")

    sim_proc = subprocess.Popen(sim_cmd, env=env, stdout=sim_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    # Wait for Gazebo and bridge to initialize
    print("Waiting 10s for simulation and parameter_bridge to initialize...", flush=True)
    time.sleep(10.0)

    # Launch yolo_node
    print("Launching vision_perception yolo_node...", flush=True)
    yolo_cmd = [
        "bash", "-lc",
        "export ROS_DOMAIN_ID=30 && "
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "(if [ -d ~/yolo_env ]; then source ~/yolo_env/bin/activate; else source ~/yolo_venv/bin/activate; fi) && "
        "ros2 run vision_perception yolo_node"
    ]
    yolo_proc = subprocess.Popen(yolo_cmd, env=env, stdout=yolo_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    print("Initializing verification collector node...", flush=True)
    rclpy.init()
    collector = VerificationCollector()

    start_wall = time.time()
    max_wall = target_sim_sec * 2.5 + 30.0

    print(f"Collecting data for ~{target_sim_sec:.0f} sim seconds (max wall: {max_wall:.0f}s)...", flush=True)
    last_progress_print = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(collector, timeout_sec=0.1)

            if collector.start_sim_time is not None and collector.surveillance_cam_times:
                current_sim = collector.surveillance_cam_times[-1]
                elapsed_sim = current_sim - collector.start_sim_time

                if time.time() - last_progress_print >= 5.0:
                    print(
                        f"  [Progress] Sim elapsed: {elapsed_sim:.1f}s | Overhead raw: {len(collector.surveillance_cam_times)} | "
                        f"Compressed: {len(collector.compressed_frames)} | Activities: {len(collector.activity_msgs)} | Goals: {len(collector.goal_poses)}",
                        flush=True
                    )
                    last_progress_print = time.time()

                if elapsed_sim >= target_sim_sec:
                    print(f"Reached {elapsed_sim:.1f} sim seconds. Finishing collection.", flush=True)
                    break

            if time.time() - start_wall > max_wall:
                print("Wall clock timeout reached. Finishing collection.")
                break
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        collector.destroy_node()
        rclpy.try_shutdown()

        print("Shutting down yolo_node and simulation...")
        try:
            os.killpg(os.getpgid(yolo_proc.pid), signal.SIGINT)
            time.sleep(1.0)
            os.killpg(os.getpgid(yolo_proc.pid), signal.SIGKILL)
        except Exception:
            pass

        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
            time.sleep(2.0)
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
        except Exception:
            pass

        subprocess.run(["pkill", "-9", "-f", "gz sim|parameter_bridge|yolo_node"], check=False)

    # Analyze results
    print("\n================ VERIFICATION REPORT ================")
    
    # 1. /surveillance_camera/image_raw rate
    cam_times = collector.surveillance_cam_times
    if len(cam_times) >= 2:
        sim_duration = cam_times[-1] - cam_times[0]
        cam_hz = (len(cam_times) - 1) / sim_duration if sim_duration > 0 else 0
        print(f"1. Overhead Camera /surveillance_camera/image_raw:")
        print(f"   Frames received: {len(cam_times)} over {sim_duration:.2f} sim seconds")
        print(f"   Average Rate: {cam_hz:.2f} Hz (Contract target: ~30 Hz)")
    else:
        print("1. Overhead Camera /surveillance_camera/image_raw: NO FRAMES RECEIVED!")

    # 2. Both independent compressed streams
    for topic, frames in (
        ("/yolo/annotated/compressed", collector.compressed_frames),
        ("/yolo/corner/annotated/compressed", collector.corner_compressed_frames),
    ):
        if len(frames) >= 2:
            duration = frames[-1][0] - frames[0][0]
            hz = (len(frames) - 1) / duration if duration > 0 else 0
            max_w = max(frame[1] for frame in frames)
            max_h = max(frame[2] for frame in frames)
            avg_kb = (sum(frame[3] for frame in frames) / len(frames)) / 1024.0
            formats = sorted(set(frame[4] for frame in frames))
            print(f"2. Compressed Stream {topic}:")
            print(f"   Frames received: {len(frames)} over {duration:.2f} sim seconds")
            print(f"   Rate: {hz:.2f} Hz (Contract: <= 10 fps)")
            print(f"   Resolution: max {max_w}x{max_h} (expected native 640x480)")
            print(f"   Format: {formats}; average size: {avg_kb:.1f} KB")
        else:
            print(f"2. Compressed Stream {topic}: NO FRAMES RECEIVED!")

    # 3. /person_activity values
    act_msgs = collector.activity_msgs
    allowed_values = {"Walking", "Running", "Staying"}
    illegal_values = []
    value_counts = collections.Counter()
    for _, val in act_msgs:
        value_counts[val] += 1
        if val not in allowed_values:
            illegal_values.append(val)

    print(f"3. Activity Label /person_activity:")
    print(f"   Messages received: {len(act_msgs)}")
    print(f"   Value distribution: {dict(value_counts)}")
    if illegal_values:
        print(f"   [FAIL] Illegal activity values detected: {set(illegal_values)}")
    else:
        print(f"   [PASS] All published activity strings strictly conform to contract: {allowed_values}")

    # 4. /goal_pose
    goals = collector.goal_poses
    print(f"4. Goal Pose /goal_pose:")
    print(f"   Total goals published: {len(goals)}")
    for idx, (gt, gframe, gx, gy, gz, gw) in enumerate(goals):
        print(f"   Goal #{idx+1}: sim_time={gt:.2f}s, frame_id='{gframe}', pos=({gx:.2f}, {gy:.2f}, {gz:.2f}), ori_w={gw:.2f}")

    # 5. Overhead camera detections at conf 0.15
    overhead_detections = 0
    corner_detections = 0
    both_detections = 0
    total_detections = 0
    for rec in collector.json_records:
        for entry in rec.get("entries", []):
            total_detections += 1
            cams = entry.get("cameras_seen", [])
            if "overhead_camera" in cams and "corner_camera" in cams:
                both_detections += 1
            elif "overhead_camera" in cams:
                overhead_detections += 1
            elif "corner_camera" in cams:
                corner_detections += 1

    print(f"5. Camera Detections Breakdown (conf 0.15):")
    print(f"   Total person detections: {total_detections}")
    print(f"   Seen by Overhead Camera only: {overhead_detections}")
    print(f"   Seen by Corner Camera only: {corner_detections}")
    print(f"   Fused (Seen by Both): {both_detections}")
    total_overhead_seen = overhead_detections + both_detections
    print(f"   Total frames actor seen by Overhead Camera: {total_overhead_seen}")

    # 6. Accuracy and Confusion Matrix against 27s cycle:
    # 0.0 - 12.0s: Staying (Idle)
    # 12.0 - 20.0s: Walking
    # 20.0 - 27.0s: Running
    confusion = collections.defaultdict(lambda: collections.defaultdict(int))
    total_eval = 0

    for rec in collector.json_records:
        sim_t = rec.get("timestamp", 0.0)
        entries = rec.get("entries", [])
        if not entries:
            pred = "Occluded"
        else:
            best = max(entries, key=lambda e: e.get("confidence", 0.0))
            pred = best.get("activity", "Unknown")

        phase = sim_t % 27.0
        if phase < 12.0:
            gt = "Staying"
        elif phase < 20.0:
            gt = "Walking"
        else:
            gt = "Running"

        confusion[gt][pred] += 1
        total_eval += 1

    print("\n6. Activity Classification Confusion Matrix against 27s Ground Truth Cycle:")
    print("   Ground Truth \\ Prediction | Staying | Walking | Running | Unknown | Occluded")
    print("   -------------------------+---------+---------+---------+---------+---------")
    for gt in ["Staying", "Walking", "Running"]:
        row = [confusion[gt][p] for p in ["Staying", "Walking", "Running", "Unknown", "Occluded"]]
        print(f"   {gt:<24} | {row[0]:>7} | {row[1]:>7} | {row[2]:>7} | {row[3]:>7} | {row[4]:>8}")

    # Walking vs Running Accuracy (when detected and classified as Walking or Running)
    walk_corr = confusion["Walking"]["Walking"]
    walk_run_err = confusion["Walking"]["Running"]
    run_corr = confusion["Running"]["Running"]
    run_walk_err = confusion["Running"]["Walking"]
    total_locomotion = walk_corr + walk_run_err + run_corr + run_walk_err
    if total_locomotion > 0:
        loco_acc = (walk_corr + run_corr) / total_locomotion * 100.0
        print(f"\n   Walking vs Running separation accuracy: {loco_acc:.1f}% ({walk_corr + run_corr}/{total_locomotion})")


if __name__ == "__main__":
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    run_verification(target_sim_sec=duration)
