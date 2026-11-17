#!/usr/bin/env python3
"""Activity evaluation script for Milestone 9/10.

Usage: python3 eval_activity.py <duration_sec>
Subscribes to /person_activity_json, maps sim timestamps to 19s actor ground truth:
  - 0.0s - 4.0s:  Staying (Idle)
  - 4.0s - 12.0s: Walking (~1.0 m/s)
  - 12.0s - 19.0s: Running (~2.5 m/s)
"""

import json
import os
import sys
import time
from collections import defaultdict

# Ensure ROS 2 python site-packages are available
for p in ["/opt/ros/jazzy/lib/python3.12/site-packages", "/opt/ros/jazzy/local/lib/python3.12/dist-packages"]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EvalActivityNode(Node):
    def __init__(self, target_duration: float = 40.0):
        super().__init__("eval_activity")
        self.target_duration = target_duration
        self.sub = self.create_subscription(
            String, "/person_activity_json", self.on_json, 10
        )
        self.records = []
        self.first_sim_time = None
        self.start_wall = time.time()
        self.done = False

    def on_json(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        sim_t = data.get("timestamp", 0.0)
        if self.first_sim_time is None:
            self.first_sim_time = sim_t

        entries = data.get("entries", [])
        if len(entries) == 0:
            act = "None"
            spd = 0.0
            cameras = []
        else:
            # Most prominent by confidence
            best = max(entries, key=lambda e: e.get("confidence", 0.0))
            act = best.get("activity", "Unknown")
            spd = best.get("speed_mps", best.get("speed", 0.0))
            cameras = best.get("cameras_seen", [])

        # 19s actor cycle
        phase_t = sim_t % 19.0
        if phase_t < 4.0:
            gt_phase = "Idle (Staying)"
        elif phase_t < 12.0:
            gt_phase = "Walking"
        else:
            gt_phase = "Running"

        self.records.append({
            "sim_time": sim_t,
            "phase_time": phase_t,
            "gt_phase": gt_phase,
            "pred_activity": act,
            "speed": spd,
            "cameras_seen": cameras,
        })

        elapsed_sim = sim_t - self.first_sim_time
        if elapsed_sim >= self.target_duration:
            self.done = True


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rclpy.init()
    node = EvalActivityNode(target_duration=duration)
    print(f"Recording /person_activity_json for {duration:.1f} sim seconds...")

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Timeout after 2.5x wall time in case sim is paused/very slow
            if time.time() - node.start_wall > duration * 2.5 + 30.0:
                print("Wall time limit reached, finishing recording...")
                break
    except KeyboardInterrupt:
        pass

    records = node.records
    node.destroy_node()
    rclpy.shutdown()

    if not records:
        print("ERROR: No messages received on /person_activity_json!")
        return

    print(f"\nTotal samples recorded: {len(records)}")
    first_t = records[0]["sim_time"]
    last_t = records[-1]["sim_time"]
    print(f"Sim time span: {first_t:.2f}s to {last_t:.2f}s ({last_t - first_t:.2f}s elapsed)")

    by_phase = defaultdict(list)
    for r in records:
        by_phase[r["gt_phase"]].append(r)

    print("\n--- Per-Phase Classification Results ---")
    for phase in ["Idle (Staying)", "Walking", "Running"]:
        phase_records = by_phase[phase]
        n = len(phase_records)
        if n == 0:
            print(f"Phase {phase}: No samples")
            continue

        counts = defaultdict(int)
        speeds = []
        visible_count = 0
        for r in phase_records:
            counts[r["pred_activity"]] += 1
            if r["pred_activity"] != "None":
                visible_count += 1
                speeds.append(r["speed"])

        mean_spd = sum(speeds) / len(speeds) if speeds else 0.0
        vis_pct = (visible_count / n) * 100.0

        print(f"\nPhase [{phase}] (N={n} frames):")
        print(f"  Visible in >=1 camera: {visible_count}/{n} ({vis_pct:.1f}%)")
        print(f"  Mean speed (when visible): {mean_spd:.3f} m/s")
        print("  Activity distribution:")
        for act in ["Staying", "Walking", "Running", "Unknown", "None"]:
            c = counts[act]
            pct = (c / n) * 100.0
            print(f"    {act:10s}: {c:3d} ({pct:5.1f}%)")


if __name__ == "__main__":
    main()
