#!/usr/bin/env python3
"""Automated measurement tool for Milestone 10 performance benchmarks.

Usage: python3 measure_setup.py [--headless true/false] [--yolo true/false] [--duration 60]
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class TopicCounter(Node):
    def __init__(self):
        super().__init__("perf_counter")
        self.cam1_count = 0
        self.cam2_count = 0
        from rclpy.qos import qos_profile_sensor_data
        self.sub1 = self.create_subscription(
            Image, "/surveillance_camera/image_raw", self.cb1, qos_profile_sensor_data
        )
        self.sub2 = self.create_subscription(
            Image, "/corner_camera/image_raw", self.cb2, qos_profile_sensor_data
        )

    def cb1(self, msg):
        self.cam1_count += 1

    def cb2(self, msg):
        self.cam2_count += 1

def parse_gz_stats(output_text):
    """Extract real_time_factor from gz topic -e -t /stats output."""
    m = re.search(r"real_time_factor:\s*([\d\.\+\-eE]+)", output_text)
    if m:
        try:
            return float(m.group(1))
        except ValueError:
            pass
    return None

def get_gz_rtf():
    try:
        res = subprocess.run(
            ["gz", "topic", "-e", "-t", "/stats", "-n", "1"],
            capture_output=True, text=True, timeout=3
        )
        return parse_gz_stats(res.stdout)
    except Exception:
        return None

def get_process_cpu():
    """Sample CPU% for key simulation and perception processes."""
    procs = {
        "gz_server": [],
        "gz_gui": [],
        "image_bridge": [],
        "parameter_bridge": [],
        "yolo_inference": [],
    }
    try:
        res = subprocess.run(
            ["ps", "-eo", "%cpu,cmd"],
            capture_output=True, text=True, timeout=3
        )
        for line in res.stdout.strip().splitlines()[1:]:
            parts = line.strip().split(None, 1)
            if len(parts) < 2:
                continue
            try:
                cpu = float(parts[0])
            except ValueError:
                continue
            cmd = parts[1]
            if ("gz sim" in cmd or "gz-sim-server" in cmd) and "gui" not in cmd and "measure_setup" not in cmd:
                procs["gz_server"].append(cpu)
            elif "gz-sim-gui" in cmd or "gz sim gui" in cmd or "libgz-sim-gui" in cmd:
                procs["gz_gui"].append(cpu)
            elif "image_bridge" in cmd:
                procs["image_bridge"].append(cpu)
            elif "parameter_bridge" in cmd:
                procs["parameter_bridge"].append(cpu)
            elif "inference.py" in cmd:
                procs["yolo_inference"].append(cpu)
    except Exception:
        pass

    return {k: (sum(v) if v else 0.0) for k, v in procs.items()}

def get_gpu_stats():
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,utilization.gpu", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=3
        )
        line = res.stdout.strip().splitlines()[0]
        mem, util = line.split(",")
        return float(mem.strip()), float(util.strip())
    except Exception:
        return None, None

def get_topic_hz(topic, window=4.0):
    try:
        proc = subprocess.Popen(
            ["bash", "-c", f"source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic hz {topic}"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            preexec_fn=os.setsid
        )
        time.sleep(window)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            out, _ = proc.communicate(timeout=2)
        except Exception:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            out, _ = proc.communicate()
        rates = re.findall(r"average rate:\s*([\d\.]+)", out)
        if rates:
            return float(rates[-1])
    except Exception:
        pass
    return 0.0

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", default="true")
    parser.add_argument("--yolo", default="false")
    parser.add_argument("--duration", type=int, default=60)
    args = parser.parse_args()

    cmd = [
        "ros2", "launch", "surveillance_sim", "surveillance_world.launch.py",
        f"headless:={args.headless}",
        f"yolo:={args.yolo}"
    ]

    print(f"=== Starting measurement: headless={args.headless}, yolo={args.yolo}, duration={args.duration}s ===")
    launch_proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        preexec_fn=os.setsid
    )

    rtf_samples = []
    cpu_records = []
    gpu_records = []
    yolo_perf_records = []

    rclpy.init()
    counter_node = TopicCounter()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(counter_node), daemon=True)
    spin_thread.start()

    # Let sim spin up for 12 seconds
    print("Waiting 12s for initial spinup...")
    time.sleep(12)

    # Reset counts for steady-state measurement
    counter_node.cam1_count = 0
    counter_node.cam2_count = 0
    steady_start = time.time()

    start_time = time.time()
    while time.time() - start_time < args.duration:
        # Sample RTF
        rtf = get_gz_rtf()
        if rtf is not None:
            rtf_samples.append(rtf)

        # Sample CPU
        cpu = get_process_cpu()
        cpu_records.append(cpu)

        # Sample GPU
        mem, util = get_gpu_stats()
        if mem is not None:
            gpu_records.append((mem, util))

        time.sleep(3)

    steady_elapsed = time.time() - steady_start
    cam1_rate = counter_node.cam1_count / steady_elapsed if steady_elapsed > 0 else 0.0
    cam2_rate = counter_node.cam2_count / steady_elapsed if steady_elapsed > 0 else 0.0

    print("Finishing run, shutting down launch process...")
    try:
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
        stdout_data, _ = launch_proc.communicate(timeout=8)
    except Exception:
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
            stdout_data, _ = launch_proc.communicate(timeout=3)
        except Exception:
            stdout_data = ""

    counter_node.destroy_node()
    rclpy.shutdown()

    # Parse YOLO perf logs from output
    # Look for [YOLO Perf] Inference: 45.2 ms/frame | Achieved: 8.1 FPS
    yolo_matches = re.findall(
        r"\[YOLO Perf\] Inference:\s*([\d\.]+)\s*ms/frame\s*\|\s*Achieved:\s*([\d\.]+)\s*FPS",
        stdout_data
    )

    print("\n================ BENCHMARK RESULTS ================")
    steady_rtf = [r for r in rtf_samples if r > 0.05]
    avg_rtf = (sum(steady_rtf) / len(steady_rtf)) if steady_rtf else (sum(rtf_samples) / len(rtf_samples) if rtf_samples else 0.0)
    print(f"RTF: {avg_rtf:.3f} (N={len(steady_rtf)} steady samples, range: {min(steady_rtf or [0]):.3f} - {max(steady_rtf or [0]):.3f})")

    # CPU averages
    if cpu_records:
        avg_gz_server = sum(r["gz_server"] for r in cpu_records) / len(cpu_records)
        avg_gz_gui = sum(r["gz_gui"] for r in cpu_records) / len(cpu_records)
        avg_img_bridge = sum(r["image_bridge"] for r in cpu_records) / len(cpu_records)
        avg_param_bridge = sum(r["parameter_bridge"] for r in cpu_records) / len(cpu_records)
        avg_yolo = sum(r["yolo_inference"] for r in cpu_records) / len(cpu_records)
        print("CPU Usage (% of 1 core):")
        print(f"  gz sim server:    {avg_gz_server:.1f}%")
        if args.headless == "false":
            print(f"  gz sim GUI:       {avg_gz_gui:.1f}%")
        print(f"  image_bridge:     {avg_img_bridge:.1f}%")
        print(f"  parameter_bridge: {avg_param_bridge:.1f}%")
        if args.yolo == "true":
            print(f"  yolo_inference:   {avg_yolo:.1f}%")

    print("Camera Rates:")
    print(f"  /camera/image_raw:  {cam1_rate:.2f} Hz")
    print(f"  /camera2/image_raw: {cam2_rate:.2f} Hz")

    if gpu_records:
        avg_mem = sum(r[0] for r in gpu_records) / len(gpu_records)
        avg_util = sum(r[1] for r in gpu_records) / len(gpu_records)
        print(f"GPU (Quadro P1000): {avg_mem:.0f} MB VRAM used, {avg_util:.1f}% utilization")

    if yolo_matches:
        inf_times = [float(m[0]) for m in yolo_matches]
        fps_vals = [float(m[1]) for m in yolo_matches]
        print("YOLO Metrics:")
        print(f"  Mean inference time: {sum(inf_times)/len(inf_times):.1f} ms/frame")
        print(f"  Achieved FPS:        {sum(fps_vals)/len(fps_vals):.1f} FPS")

    print("===================================================\n")

if __name__ == "__main__":
    main()
