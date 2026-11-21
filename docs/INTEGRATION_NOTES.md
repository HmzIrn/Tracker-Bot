# Surveillance Simulation and Perception Integration Notes

## 1. Package Boundaries

- `surveillance_sim` owns the Gazebo Harmonic world, actor, cameras, and Gazebo-to-ROS bridges.
- `vision_perception` owns `yolo_node`, camera calibration, tracking, activity classification, goal poses, and annotated image streams.
- This workspace does not create `/map` or include a robot, navigation, localization, or mapping stack. The consuming integration supplies a compatible map.

## 2. World and Cameras

The final SDF is a 16 m x 16 m room with perimeter walls, two pillars, two crates, two cameras, and one animated actor. The former central partition is absent because it blocked the corner camera; maps must correspond to this partition-free world.

The actor repeats a 27 s trajectory:

| Segment | Sim-time interval | Waypoint speed |
| :--- | :--- | :--- |
| Idle at `(0,0)` | 0-12 s | 0 m/s |
| Walk in | 12-16 s | 0.875 m/s |
| Run | 16-16.833 s, 16.833-19 s | 3.000, 3.138 m/s |
| Walk return | 19-27 s | 1.100, 1.000, 1.000, 1.000, 1.000 m/s |

The `staying_thresh=0.3 m/s` and `running_thresh=1.75 m/s` defaults are unchanged. Path tests check room bounds, obstacle clearance, and the corner camera frustum. The actor's `stand.dae`, `walk.dae`, and `run.dae` assets are referenced from Gazebo Fuel URLs, so the first world launch may need network access or a populated Gazebo model cache.

| Camera | Pose `(x,y,z,roll,pitch,yaw)` | Image / HFOV | SDF rate | ROS image topic |
| :--- | :--- | :--- | ---: | :--- |
| Overhead | `(0,0,12.2,0,1.5708,1.5708)` | 640x480 / 1.50 rad | 30 Hz | `/surveillance_camera/image_raw` |
| Corner | `(-7,-7,2.3,0,0.25,0.7854)` | 640x480 / 1.05 rad | 10 Hz | `/corner_camera/image_raw` |

`vision_perception/config/cameras.yaml` supplies pose, image size, HFOV, projection plane, and ground-point selection. Overhead projection intersects `plane_z=1.5` at the bbox center; corner projection intersects `plane_z=0.0` at the bbox bottom-center. Runtime dimension checks and `test_camera_sdf_drift.py` guard SDF/YAML drift.

## 3. ROS Interface Contract

| Topic | Type | Contract |
| :--- | :--- | :--- |
| `/person_activity` | `std_msgs/msg/String` | Exact values `"Walking"`, `"Running"`, `"Staying"`; priority is Staying > Running > Walking. Publish while a person is classified and nothing when no person is classified. |
| `/yolo/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | Overhead-only native 640x480 JPEG, quality 75, no more than 10 fps. |
| `/yolo/corner/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | Corner-only native 640x480 JPEG, quality 75, no more than 10 fps. |
| `/yolo/annotated/raw`, `/yolo/corner/annotated/raw` | `sensor_msgs/msg/Image` | Optional native raw outputs; disabled by default. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | One event per continuous Staying episode of at least 5 s; `frame_id=map`, `z=0`, `w=1`, at least 15 s global cooldown, and a free-space cell on `/map`. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Input to `yolo_node`; must be provided by the consuming integration with transient-local durability. No map means no goal: the episode remains armed and retries while the person is still Staying. |

Both annotated streams contain native camera detections and projected fused global-track markers. The node forces `use_sim_time=True` and uses image-header timestamps for activity and dwell calculations. Use `bridge_clock:=true` when other ROS timers or nodes require `/clock`. `max_fps` defaults to 2.0 wall-clock frames/s and is capped at 10.

## 4. Setup, Registration, and Run

### Requirements

- WSL2 Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic, `colcon`, `ros_gz_sim`, `ros_gz_bridge`, and the ROS message dependencies declared in the two package manifests.
- YOLO runtime Python packages: NumPy below 2.0, OpenCV, PyYAML, PyTorch, and Ultralytics. The node searches `~/yolo_env` before `~/yolo_venv`; inference currently uses the CPU.
- Ultralytics weights `yolov8n.pt`. If not already cached, the first YOLO start may download them. Do not add model weights to the repository.
- GUI tasks additionally need a working WSLg display; headless launch does not require the GUI.

### Register once after cloning or moving the workspace

Run from the workspace root in WSL:

```bash
bash scripts/register_with_ros.sh
```

This builds with `colcon build --symlink-install`, writes the workspace overlay path to `~/.survellence_sim.bash`, and adds a marked source block to `~/.bashrc`. The first edit backs up the existing file to `~/.bashrc.survellence_sim.bak`. The script is safe to rerun, supports `--no-build` and reversible `--remove`, and does not set `ROS_DOMAIN_ID`.

The VS Code task is named **`Survellence_Sim: Register with ROS 2 (run once)`** and has native Linux and Windows/WSL blocks. Tasks and `run_sim.ps1` set `ROS_DOMAIN_ID=30`; for manual commands, set it in the shell:

```bash
source ~/.survellence_sim.bash
export ROS_DOMAIN_ID=30
colcon build --symlink-install
pytest -q
```

Launch the world with the perception node:

```bash
ros2 launch surveillance_sim surveillance_world.launch.py yolo:=true bridge_clock:=true
```

Or start the world and node separately:

```bash
ros2 launch surveillance_sim surveillance_world.launch.py headless:=true bridge_clock:=true
ros2 run vision_perception yolo_node
```

The app's node command is the bare `ros2 run vision_perception yolo_node` (no arguments or stdin). Stop it with SIGINT; if needed, follow with SIGKILL after 6 seconds.

## 5. Map Coordinates and Free-Space Checks

`world_to_map: [tx, ty, yaw_radians]` transforms the projected Gazebo world point into the `map` frame:

```text
x_map = tx + cos(yaw)*x_world - sin(yaw)*y_world
y_map = ty + sin(yaw)*x_world + cos(yaw)*y_world
```

If world and map axes align, set yaw to zero; for example, a map-frame translation of `(1.25,-2.0)` uses `world_to_map: [1.25, -2.0, 0.0]`. The value is not the OccupancyGrid origin. Calibrate it by comparing known world points with their coordinates in the supplied map.

For goal validation, the node uses map origin position, resolution, width, height, and row-major cell order. **The free-space nudge currently assumes map-origin yaw is zero.** A non-free target is moved to the nearest free cell within 0.5 m; a missing map defers the goal. The test map used in verification had origin `(-5,-5)`, resolution `0.1 m`, size `100x100`, and all cells free.

## 6. Verification Results

### Calibration and earlier detector experiments

- The camera/SDF test compares both camera poses, HFOVs, and 640x480 resolutions. Integer-pixel geometric round-trip errors against the room-center and near-wall waypoints were 0.00004 m overhead and 0.06817 m corner; these are projection quantization results, not detector localization accuracy.
- A2's saved, partially occluded frame pair is available at [overhead](../a2_overhead_frame.jpg) and [corner](../a2_corner_frame.jpg). Actor extents were about 15x16 px overhead and 14x80 px corner. At `conf=0.15`, overhead yielded zero detections for v8n/s/m at imgsz 640/960/1280. Corner detections were: v8n `0/1/1`, v8s `0/1/2`, and v8m `0/0/1` at those sizes. No sensor resolution or model default was changed.
- The earlier two-cycle F3 sample (before the H2 route update) measured 55/55 corner detections for v8n/s at imgsz 640/960, with mean actor height 80.95 px. Its results are historical and not directly comparable to the current route below.

### Current route: 81-second runtime check

The headless run used the registered shell, `bridge_clock:=true`, the transient-local test map, and bare `ros2 run vision_perception yolo_node`. It collected 81.016 simulated seconds in 120.190 wall seconds (RTF 0.674); `/clock` and image-header spans differed by 0.063 s.

Exactly three goals were emitted, one during each idle interval:

| Image-stamp time | Map position `(x,y)` m | Frame and orientation |
| ---: | :--- | :--- |
| 11.683 s | `(-0.118,-0.075)` | `map`, `z=0`, `w=1` |
| 35.377 s | `(-0.109,-0.070)` | `map`, `z=0`, `w=1` |
| 61.678 s | `(-0.109,-0.064)` | `map`, `z=0`, `w=1` |

Each goal is within 0.14 m of the idle point `(0,0)`. F1's separate 43-second no-map run observed zero goals; missing-map warnings were throttled and the Staying episode remained eligible for retry.

For classification, each inference sample was assigned the closest tracked position to the SDF-interpolated actor position. Counts below are true segment by predicted Staying / Walking / Running / Unknown:

| Ground-truth segment | Staying | Walking | Running | Unknown |
| :--- | ---: | ---: | ---: | ---: |
| Idle | 67 | 15 | 0 | 2 |
| Walk in | 14 | 18 | 0 | 0 |
| Run | 0 | 15 | 4 | 6 |
| Walk return | 0 | 58 | 9 | 0 |

Corner detections occurred in 208/210 inference samples (99.0%); two idle samples had no tracked entry. The nearest actor track ID changed five times across the run, and nine inference samples contained multiple IDs.

| Stream | Messages | Wall Hz | Sim Hz |
| :--- | ---: | ---: | ---: |
| Overhead camera input | 1390 | 11.557 | 17.145 |
| Corner camera input | 436 | 3.624 | 5.377 |
| Inference telemetry | 210 | 1.874 | 2.588 |
| `/person_activity` | 100 | 0.902 | 1.335 |
| Overhead annotated JPEG | 209 | 1.872 | 2.754 |
| Corner annotated JPEG | 210 | 1.874 | 2.590 |

Both annotated topics were `jpeg`, 640x480, configured quality 75. All 100 activity messages used a valid contract string (41 Staying, 51 Walking, 8 Running). The measured average meets 1-10 Hz on simulated time; the wall-time average was 0.902 Hz at this RTF. Clarify with the app owner if the 1 Hz minimum is intended to be wall-clock rather than simulation-time.

Sampled run frames at 5 Hz show visible foot glide during the 3.0-3.14 m/s run. This was a qualitative visual check, not a measured foot-contact trajectory; actor speed or animation playback was not changed.

### Historical rate and classifier measurements

- Earlier 30 Hz overhead baseline: RTF 0.763. A temporary 15 Hz SDF copy measured RTF 0.826 (+8.3%); sim-time inference/annotated rates shifted from about 2.47 to 2.22 Hz, while wall rates stayed near 1.9 Hz. The source SDF was not changed. Confirm before any permanent camera-rate change.
- Earlier A4 sampling replay at `max_fps=2.0`: 11/33 track updates were below three samples in a 1 s window; replaying the same observations with a 2 s window reduced this to 9/33. Speed thresholds were left unchanged. The current-route confusion table above shows that Running remains poorly recognized.
- No GPU software was installed. PyTorch is built for CUDA 13.0 while the WSL driver reports capability 12.5; `torch.cuda.is_available()` is false. Inference uses CPU. A compatible driver/runtime and supported GPU would need to be provided before revisiting GPU inference.

## 7. Known Limitations and Handoff

- Overhead person detection is poor at the actor's small pixel size; fused corner detections are projected into the overhead annotation.
- The current-route classifier confuses idle/walking and detects only 4/25 run samples as Running; tracking also fragments during run segments. Keep current thresholds and defaults pending a deliberate tuning decision.
- The app-visible activity stream averaged 0.902 Hz wall time in the measured run, although it averaged 1.335 Hz simulated time. Decide which clock defines the minimum-rate contract.
- Run-clip foot glide is visible at the current actor speed. The map producer and `world_to_map` calibration belong to the consuming integration; the occupancy-grid nudge assumes zero origin yaw.
- The actor assets are external Gazebo Fuel resources, and inference is CPU-limited by the current CUDA/driver mismatch.

For another workspace, copy `src/surveillance_sim` and `src/vision_perception` as separate packages and build them together. This repository does not provide a robot launch wrapper or modify other packages.
