#!/usr/bin/env python3
"""Record perception rates and one corner frame per simulated second."""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SDF = os.path.join(ROOT, "src", "surveillance_sim", "worlds", "surveillance_world.sdf")


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Recorder(Node):
    def __init__(self, output_dir):
        super().__init__("milestone11_rate_recorder")
        self.output_dir = output_dir
        self.corner_dir = os.path.join(output_dir, "corner_frames")
        self.gait_dir = os.path.join(output_dir, "gait_frames")
        os.makedirs(self.corner_dir, exist_ok=True)
        os.makedirs(self.gait_dir, exist_ok=True)
        self.events = {
            "overhead_input": [],
            "corner_input": [],
            "activity": [],
            "inference": [],
            "overhead_annotated": [],
            "corner_annotated": [],
            "goals": [],
        }
        self.corner_frames = []
        self.gait_frames = []
        self.saved_gait_bins = set()
        self.last_saved_second = -1
        self.sim_time = 0.0
        self.first_camera = None
        self.last_camera = None
        self.last_inference_sim_time = None
        self.first_clock = None
        self.last_clock = None
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_subscription(
            Image, "/surveillance_camera/image_raw",
            lambda msg: self.on_image("overhead_input", msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, "/corner_camera/image_raw",
            lambda msg: self.on_image("corner_input", msg), qos_profile_sensor_data,
        )
        self.create_subscription(String, "/person_activity", self.on_activity, 10)
        self.create_subscription(
            String, "/person_activity_json", self.on_inference, 10
        )
        self.create_subscription(
            CompressedImage, "/yolo/annotated/compressed",
            lambda msg: self.on_compressed("overhead_annotated", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage, "/yolo/corner/annotated/compressed",
            lambda msg: self.on_compressed("corner_annotated", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)

    def record(self, channel, msg_sim_time=None, data=None):
        self.events[channel].append({
            "wall": time.monotonic(),
            "sim": self.sim_time,
            "msg_sim": msg_sim_time,
            "data": data,
        })

    def on_clock(self, msg):
        self.sim_time = stamp_seconds(msg.clock)
        sample = {"wall": time.monotonic(), "sim": self.sim_time}
        if self.first_clock is None:
            self.first_clock = sample
        self.last_clock = sample

    def on_image(self, channel, msg):
        msg_time = stamp_seconds(msg.header.stamp)
        now = time.monotonic()
        if self.first_camera is None:
            self.first_camera = {"wall": now, "sim": msg_time}
        if self.last_camera is None or msg_time >= self.last_camera["sim"]:
            self.last_camera = {"wall": now, "sim": msg_time}
        self.record(channel, msg_time)
        if channel != "corner_input":
            return

        second = int(msg_time)
        save_regular = second > self.last_saved_second
        cycle = int(msg_time // 27.0)
        cycle_time = msg_time - cycle * 27.0
        gait_bin = int((cycle_time - 16.0) / 0.2)
        gait_key = (cycle, gait_bin)
        save_gait = (
            16.0 <= cycle_time < 19.0
            and 0 <= gait_bin < 15
            and gait_key not in self.saved_gait_bins
        )
        if not save_regular and not save_gait:
            return

        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif msg.encoding != "bgr8":
            self.get_logger().error(f"Unsupported corner encoding: {msg.encoding}")
            return

        if save_regular:
            path = os.path.join(self.corner_dir, "corner_%04d.jpg" % second)
            if not cv2.imwrite(path, image, [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
                self.get_logger().error(f"Failed to save corner frame {path}")
            else:
                self.corner_frames.append({"sim_time": msg_time, "path": path})
                self.last_saved_second = second
        if save_gait:
            path = os.path.join(
                self.gait_dir, "run_cycle_%02d_frame_%02d.jpg" % gait_key
            )
            if not cv2.imwrite(path, image, [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
                self.get_logger().error(f"Failed to save gait frame {path}")
            else:
                self.gait_frames.append({"sim_time": msg_time, "path": path})
                self.saved_gait_bins.add(gait_key)

    def on_activity(self, msg):
        self.record(
            "activity",
            self.last_inference_sim_time,
            data=msg.data,
        )

    def on_inference(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid telemetry JSON: {exc}")
            return
        self.last_inference_sim_time = payload.get("timestamp")
        self.record("inference", self.last_inference_sim_time, payload)

    def on_compressed(self, channel, msg):
        image = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        if image is None:
            self.get_logger().error(f"Could not decode JPEG from {channel}")
            dimensions = None
        else:
            dimensions = {"width": int(image.shape[1]), "height": int(image.shape[0])}
        self.record(channel, stamp_seconds(msg.header.stamp), {
            "format": msg.format,
            "bytes": len(msg.data),
            "dimensions": dimensions,
        })

    def on_goal(self, msg):
        self.record("goals", stamp_seconds(msg.header.stamp), {
            "frame_id": msg.header.frame_id,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "w": msg.pose.orientation.w,
        })


def update_rate_summary(events):
    if len(events) < 2:
        return {"messages": len(events), "wall_hz": 0.0, "sim_hz": 0.0}
    wall_elapsed = events[-1]["wall"] - events[0]["wall"]
    message_times = [
        event["msg_sim"] for event in events if event["msg_sim"] is not None
    ]
    if len(message_times) >= 2:
        sim_elapsed = message_times[-1] - message_times[0]
    else:
        sim_elapsed = events[-1]["sim"] - events[0]["sim"]
    intervals = len(events) - 1
    return {
        "messages": len(events),
        "wall_hz": intervals / wall_elapsed if wall_elapsed > 0 else 0.0,
        "sim_hz": intervals / sim_elapsed if sim_elapsed > 0 else 0.0,
        "wall_seconds": wall_elapsed,
        "sim_seconds": sim_elapsed,
    }


def summarize_compressed_events(events):
    formats = sorted({event["data"]["format"] for event in events})
    dimensions = sorted({
        (
            event["data"]["dimensions"]["width"],
            event["data"]["dimensions"]["height"],
        )
        for event in events
        if event["data"]["dimensions"] is not None
    })
    return {
        "messages": len(events),
        "formats": formats,
        "dimensions": [
            {"width": width, "height": height} for width, height in dimensions
        ],
        "mean_bytes": (
            sum(event["data"]["bytes"] for event in events) / len(events)
            if events else 0.0
        ),
    }


def stop_owned_process(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=6)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def make_rate_limited_world(rate, temp_dir):
    tree = ET.parse(SDF)
    model = tree.getroot().find(".//model[@name='overhead_camera']")
    sensor = model.find(".//sensor[@name='overhead_camera']")
    update_rate = sensor.find("update_rate")
    update_rate.text = str(rate)
    path = os.path.join(temp_dir, "surveillance_world_%dhz.sdf" % rate)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-seconds", type=float, required=True)
    parser.add_argument("--with-map", action="store_true")
    parser.add_argument("--overhead-hz", type=int, choices=(15, 30), default=30)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--wall-guard", type=float)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    os.environ["ROS_DOMAIN_ID"] = "30"
    env = os.environ.copy()
    env.setdefault("GALLIUM_DRIVER", "d3d12")
    env.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")
    rclpy.init()
    recorder = Recorder(args.output_dir)
    simulation = yolo = map_helper = None
    temp_world = tempfile.TemporaryDirectory() if args.overhead_hz == 15 else None
    try:
        world_path = SDF
        if temp_world is not None:
            world_path = make_rate_limited_world(15, temp_world.name)
        if args.with_map:
            map_helper = subprocess.Popen(
                [sys.executable, os.path.join(os.path.dirname(__file__), "publish_test_map.py")],
                env=env, start_new_session=True,
            )
        simulation = subprocess.Popen(
            [
                "ros2", "launch", "surveillance_sim", "surveillance_world.launch.py",
                "headless:=true", "bridge_clock:=true", "world:=" + world_path,
            ],
            env=env, start_new_session=True,
        )
        yolo = subprocess.Popen(
            ["ros2", "run", "vision_perception", "yolo_node"],
            env=env, start_new_session=True,
        )
        wall_guard = args.wall_guard or args.sim_seconds * 3.5 + 90.0
        wall_start = time.monotonic()
        while time.monotonic() - wall_start < wall_guard:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if recorder.first_camera and recorder.last_camera:
                elapsed = recorder.last_camera["sim"] - recorder.first_camera["sim"]
                if elapsed >= args.sim_seconds:
                    break
            if simulation.poll() is not None or yolo.poll() is not None:
                raise RuntimeError("Gazebo or yolo_node exited during measurement")
        else:
            raise TimeoutError(
                "Only %.2f of %.2f requested sim seconds were observed"
                % (
                    recorder.last_camera["sim"] - recorder.first_camera["sim"]
                    if recorder.first_camera and recorder.last_camera else 0.0,
                    args.sim_seconds,
                )
            )

        sim_elapsed = recorder.last_camera["sim"] - recorder.first_camera["sim"]
        wall_elapsed = recorder.last_camera["wall"] - recorder.first_camera["wall"]
        clock_elapsed = (
            recorder.last_clock["sim"] - recorder.first_clock["sim"]
            if recorder.first_clock and recorder.last_clock else 0.0
        )
        rates = {
            name: update_rate_summary(events)
            for name, events in recorder.events.items()
        }
        result = {
            "requested_sim_seconds": args.sim_seconds,
            "sim_seconds": sim_elapsed,
            "wall_seconds": wall_elapsed,
            "rtf": sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0.0,
            "clock_sim_seconds": clock_elapsed,
            "clock_camera_drift_seconds": sim_elapsed - clock_elapsed,
            "overhead_hz": args.overhead_hz,
            "with_map": args.with_map,
            "rates": rates,
            "annotated_streams": {
                name: summarize_compressed_events(recorder.events[name])
                for name in ("overhead_annotated", "corner_annotated")
            },
            "activity_messages": [
                {
                    "wall": event["wall"],
                    "sim_time": event["msg_sim"],
                    "data": event["data"],
                }
                for event in recorder.events["activity"]
            ],
            "inference_samples": [
                {
                    "wall": event["wall"],
                    "sim_time": event["msg_sim"],
                    "entries": event["data"]["entries"],
                }
                for event in recorder.events["inference"]
            ],
            "goals": recorder.events["goals"],
            "corner_frames": recorder.corner_frames,
            "gait_frames": recorder.gait_frames,
        }
        output_path = os.path.join(args.output_dir, "measurement.json")
        with open(output_path, "w", encoding="utf-8") as output:
            json.dump(result, output, indent=2)
        print(json.dumps(result, indent=2), flush=True)
    finally:
        stop_owned_process(yolo)
        stop_owned_process(map_helper)
        stop_owned_process(simulation)
        recorder.destroy_node()
        rclpy.try_shutdown()
        if temp_world is not None:
            temp_world.cleanup()


if __name__ == "__main__":
    main()
