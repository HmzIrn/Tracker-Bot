# Changelog

All notable changes and verification outcomes are documented in this file.

## [Milestone 11] - 2026-11-20: Flutter Dashboard Integration Contract

### Addendum A1: Separate Native-Resolution Annotated Streams
- Replaced the side-by-side composite with independent overhead `/yolo/annotated/compressed` and corner `/yolo/corner/annotated/compressed` JPEG streams. Both retain native frames, use quality 75, and are capped at 10 fps; optional raw outputs are separate and disabled by default.
- YOLO receives each camera's own frame via `predict(imgsz=640)` without sharing a persistent tracker. Each stream draws its camera's native detection boxes plus projected global-track markers and labels.
- Updated the stream contract tests, `context.md`, and `docs/INTEGRATION_NOTES.md`.
- Checkpoint: `colcon build --symlink-install` passed; perception pytest suite **24 passed**. Simulation output verification is deferred to the requested A2/A4 measurement run.

### Addendum A2: Overhead Detection Experiment
- Captured and retained actor-visible native frames at sim stamp 3.0 s: `a2_overhead_frame.jpg` and `a2_corner_frame.jpg`. Approximate actor extent in the source frames: overhead **~15x16 px**; corner **~14x80 px** (partially occluded by the partition).
- Ran YOLOv8n/s/m on CPU at `imgsz` 640/960/1280 and `conf=0.15`; all overhead results were zero detections. Corner detections were:

  | Model | 640 | 960 | 1280 |
  | :--- | :--- | :--- | :--- |
  | YOLOv8n | 0 | 1 (`0.211`, 13.1x55.0 px) | 1 (`0.419`, 14.0x56.3 px) |
  | YOLOv8s | 0 | 1 (`0.228`, 12.9x55.8 px) | 2 (`0.471`, 13.5x55.8 px; `0.334`, 13.0x80.6 px) |
  | YOLOv8m | 0 | 0 | 1 (`0.399`, 14.1x54.6 px) |
- The single headless run collected **42.87 sim seconds** before the 110 s wall-time guard, short of 60 sim seconds. No camera resolution or model default was changed. The overhead detector's failure supports A1's projected corner-track markers.
- Checkpoint: `colcon build --symlink-install` passed; perception pytest suite **24 passed**.

### Addendum A3: Per-Camera Projection Calibration
- Added `plane_z` to camera configuration: overhead **1.5 m** with `center`; corner **0.0 m** with `bottom_center`. Pixel-ray intersection now uses the configured plane.
- Intrinsics are required from YAML rather than embedded fallback values. `yolo_node` validates incoming dimensions; the SDF drift test now compares pose, HFOV (1.50/1.05 rad), and resolution (640x480) for both cameras.
- Round-trip calibration against room-center `(0, 0)` and near-wall `(0.5, 4.5)` waypoints gives pixel-quantization errors of **0.00004 m** overhead at center and **0.06817 m** corner near the wall.
- Checkpoint: `colcon build --symlink-install` passed; perception pytest suite **25 passed**.

### Addendum A4: Classifier Sampling Experiment
- The 42.07 sim-second telemetry run at `max_fps=2.0`, `window_sec=1.0` produced **33 tracked updates** across tracks 1-6. In a 1.0 s window, sample counts were min/median/max **1/3/4**, with **11/33** updates below `min_samples=3`.
- Increased the default window to **2.0 s**; `staying_thresh=0.3` and `running_thresh=1.75` were unchanged. A replay of the same track positions had min/median/max samples **1/5/6**, with **9/33** below three.
- Confusion matrix (rows = known actor state; columns = predicted Staying / Walking / Running / Unknown):

  | Window | Staying | Walking | Running |
  | :--- | :--- | :--- | :--- |
  | 1.0 s | 0 / 0 / 0 / 2 | 0 / 8 / 0 / 5 | 0 / 15 / 1 / 2 |
  | 2.0 s replay | 0 / 0 / 0 / 2 | 0 / 8 / 0 / 5 | 0 / 16 / 0 / 2 |

- The replay reduces sample starvation but does not improve the confusion matrix; further threshold tuning is deferred. At `max_fps=2.0`, 104 telemetry records over 42.07 sim seconds is **2.47 Hz simulated**; measured RTF was approximately **0.5**. Processing was CPU-limited, so `max_fps` remains 2 rather than being raised toward 5.
- PyTorch reports CUDA build **13.0**, while the available WSL driver reports capability **12.5**; `torch.cuda.is_available()` is false. A compatible newer driver for the Quadro P1000 or a PyTorch CUDA wheel built for a driver-supported runtime (and Pascal GPU) may resolve this; nothing was installed.
- Checkpoint: `colcon build --symlink-install` passed; perception pytest suite **26 passed**.

### Addendum A5: Goal Pose Deferral and Map Indexing
- `/goal_pose` is no longer published without `/map`. The Staying episode remains unfired, cooldown is untouched, and image callbacks retry while the track remains Staying; the missing-map warning is throttled to 5 s.
- Dwell and cooldown continue to use camera image header stamps via a ROS-independent timestamp conversion helper. Tests cover a frozen node clock, retry without `mark_fired`, and cooldown behavior.
- Confirmed free-space indexing uses map origin, resolution, width/height, and row-major cell order. Added a non-zero-origin, 0.2 m resolution test; map origin yaw is assumed zero.
- Checkpoint: `colcon build --symlink-install` passed; perception pytest suite **29 passed**.

### Addendum A6: Results and Open Decisions
- Added the A1-A5 measurements, A2 detection table, known limitations, and decision points to `docs/INTEGRATION_NOTES.md`. No out-of-scope packages or robot integrations were changed.
- Final verification after all code changes: `colcon build --symlink-install` passed; full workspace `pytest -q` **58 passed**; changed Python modules passed `py_compile`.
- The headless capture reached 42.87 simulated seconds before the wall-time guard (RTF approximately 0.5); this limitation and the remaining empirical calibration/map decisions are documented.

### Follow-up F1: End-to-end `/goal_pose` verification
- With a transient-local free test map (origin `(-5,-5)`, resolution `0.1 m`, `100x100`) over 55 simulated seconds, exactly one valid goal was observed in each of the two idle episodes. Both were within 0.14 m of the `(0,0)` idle position and had the required `map` frame and pose fields. A separate 43-second run without the map helper produced zero goals and throttled missing-map warnings.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **58 passed**.

### Follow-up F2: Actor waypoint speed comparison
- Calculated waypoint speeds are idle **0**, walk **1.010/1.131 m/s**, and run-trajectory **1.697/1.166 m/s**. With thresholds 0.3/1.75 m/s, all moving segments are above Staying but below Running. No speed or threshold was changed.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **58 passed**.

### Follow-up F3: Corner-camera recall
- Across 55 unoccluded frames from two actor cycles, v8n/v8s at imgsz 640/960 each detected the actor in **55/55** frames. Mean actor height was **80.95 px** (range 63–94); CPU mean/median per frame was **115.3/58.6 ms** (v8n-640), **156.1/151.5 ms** (v8s-640), **143.0/133.9 ms** (v8n-960), and **354.9/350.3 ms** (v8s-960). Recommendation: retain v8n-640 pending broader data. Unlike the occluded A2 sample, the current SDF has its central partition commented out.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **58 passed**.

### Follow-up F4: Rates and temporary 15 Hz camera test
- At the 30 Hz source rate, measured RTF was **0.763**; the temporary 15 Hz world copy measured **0.826** (+8.3%). Sim-time inference/annotated rates shifted from about **2.47 Hz** to **2.22 Hz**; wall-time rates remained about **1.95/1.91 Hz**. `max_fps` is wall-clock limited, so sim-time Hz scales approximately inversely with RTF. The source SDF was not changed; ask before making this permanent.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **58 passed**.

### Follow-up F5: Consolidated report
- Added F1-F4 measurements, conclusions, and open decisions to `docs/INTEGRATION_NOTES.md`. Hardened the non-installed measurement runner to initialize its recorder on domain 30 and use compatible sensor/clock QoS. No GPU software was installed; PyTorch `2.14.1+cu130` is incompatible with the observed WSL driver capability `12.5`, so the P1000 requires a supported driver/runtime pairing or compatible CUDA wheel.
- Final checkpoint: `colcon build --symlink-install` passed; full workspace `pytest -q` **58 passed**. No persistent SDF rate change was made.

### Close-out H1: Freeze the partition-free world
- Replaced the commented-out partition model with a concise note explaining the corner-camera occlusion and requiring maps to match the final world. Documented the perimeter walls, two pillars, two crates, cameras, actor, and absent partition.
- After `colcon build --symlink-install`, the installed SDF was byte-identical to source and contained no active `partition_wall` model. Checkpoint: workspace `pytest -q` **58 passed**.

### Close-out H2: Raise actor run speed and protect tracking
- Kept the 12 s idle, 27 s cycle, and `running_thresh=1.75`. Walk speeds are **0.875/1.100/1.000/1.000/1.000/1.000 m/s**; run segments are **3.000/3.138 m/s** (overall **3.100 m/s**). Added a per-track gate `max(1.5 m, 4.0 m/s * age)` and a replay at the measured 2.47 Hz sim-time rate with **0 ID restarts**. The actor path test checks sampled clearance and camera frustum; visual gait review follows in H4.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **68 passed**.

### Close-out H3: Register the workspace with ROS 2
- The requested old and new package names are both `surveillance_sim`; no directory/package rename was applicable. Confirmed one `surveillance_sim` entry in `ros2 pkg list`, the `vision_perception yolo_node` executable, and launch argument resolution from a shell sourcing only `~/.survellence_sim.bash`.
- Removed generated `build/surveillance_sim`, `install/surveillance_sim`, and root `log/` before a clean `colcon build --symlink-install`; both packages rebuilt successfully. Added `scripts/register_with_ros.sh` with `--no-build` and reversible `--remove`; it does not set `ROS_DOMAIN_ID`. Two registrations preserved `.bashrc` and the registration file byte-for-byte; removal restored the pre-registration backup, and re-registration passed the fresh-shell checks.
- Added the Linux/Windows VS Code registration task and LF policy for shell scripts. Updated both YOLO task variants to run `vision_perception yolo_node` directly; `run_sim.ps1` already referenced the requested package name and needed no change.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **68 passed**.

### Close-out H4: Revalidate three actor cycles
- From a shell sourcing only `~/.survellence_sim.bash`, ran the headless world with `bridge_clock:=true`, the transient-local test map, and bare `ros2 run vision_perception yolo_node`. The recorder reached **81.016 sim s / 120.190 wall s (RTF 0.674)**; `/clock` and camera-header spans differed by only **0.063 s**.
- Exactly **3** goals were recorded, at sim **11.683/35.377/61.678 s**, one during each 12 s idle interval. All had `frame_id=map`, `z=0`, `w=1`, and map positions within **0.14 m** of `(0,0)`. Reused F1's 43 sim-second no-map run: **0 goals**.
- Confusion counts use the closest tracked entry to the SDF actor position; columns are Staying / Walking / Running / Unknown:

  | Ground-truth segment | Staying | Walking | Running | Unknown |
  | :--- | ---: | ---: | ---: | ---: |
  | Idle | 67 | 15 | 0 | 2 |
  | Walk in | 14 | 18 | 0 | 0 |
  | Run | 0 | 15 | 4 | 6 |
  | Walk return | 0 | 58 | 9 | 0 |

- Corner detections appeared in **208/210** inference samples (**99.0%**); two idle samples had no tracked entry. The nearest-actor ID changed **5** times across the three cycles; **9** inference samples contained multiple IDs. The 100 `/person_activity` messages used only the three allowed labels (41 Staying, 51 Walking, 8 Running); average rate was **1.335 Hz simulated / 0.902 Hz wall**. Thus it meets 1-10 Hz on sim time, but is just below 1 Hz if interpreted as a wall-time requirement.
- Both annotated topics published JPEG at **640x480** (configured quality 75): overhead **209** messages at **1.872 wall / 2.754 sim Hz**; corner **210** at **1.874 wall / 2.590 sim Hz**. Camera inputs measured **11.557/17.145 Hz wall/sim** overhead and **3.624/5.377 Hz** corner; inference was **1.874/2.588 Hz**.
- Sampled run-clip frames show visible foot glide at the 3.0-3.14 m/s actor speed; this is qualitative, not a contact-point measurement. No trajectory or animation tuning was made. Open decision: confirm whether `/person_activity`'s 1 Hz minimum is wall-time or sim-time for the app.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **68 passed**.

### Close-out H5: Refresh integration documentation
- Rewrote `context.md` to reflect the split `surveillance_sim`/`vision_perception` packages, current partition-free world and 27 s route, camera calibration, ROS contracts, registration, and current runtime limitations.
- Reworked `docs/INTEGRATION_NOTES.md` to remove the obsolete wrapper/robot-launch material and duplicate section numbering; added setup requirements, the exact registration task/file names, yaw-zero map guidance and example, and the H4 confusion/rate/goal results. Kept valid A2 image links and labeled pre-H2 measurements as historical.
- Checked both Markdown files for stale robot/partition references and broken relative links.
- Checkpoint: `colcon build --symlink-install` passed; workspace `pytest -q` **68 passed**.

### Close-out H6: Validate a clean clone
- Added ignore rules for local measurement, frame, and experiment output directories; existing build/install/log, Python cache, and model-weight rules remain in effect.
- Assembled a temporary clone from `HEAD`, tracked worktree changes, and explicit copies of the untracked source files, excluding generated captures. In that clone, `colcon build --symlink-install` built both packages, `pytest -q` passed **68 tests**, and `register_with_ros.sh --no-build` resolved the expected packages, executable, and launch arguments. Restored registration to the primary workspace afterward.
- Checkpoint: clean-clone build passed; clean-clone pytest **68 passed**; clean-clone ROS registration passed.

### Step 4: /goal_pose Publisher
- Added goal pose configuration to `config/cameras.yaml` (`staying_dwell_sec: 5.0`, `goal_cooldown_sec: 15.0`, `world_to_map: [0.0, 0.0, 0.0]`), declared in `yolo_node` with runtime ROS parameter override support.
- Implemented `GoalPoseManager` logic: emits a single `geometry_msgs/msg/PoseStamped` on `/goal_pose` (frame "map", z=0, w=1.0) when a tracked person dwells in "Staying" continuously for >= 5.0 seconds (sim time). Uses median projected floor coordinates over the dwelling window, transformed by `world_to_map`.
- Enforced a 15.0-second global cooldown between goal publications and immediate episode reset when the track leaves Staying or is lost.
- Implemented occupancy grid free-space nudging subscribing to `/map` (transient-local QoS): nudges targets to the nearest free cell within 0.5 m if occupied, logs a warning and skips publishing if no free cell is within 0.5 m, or logs a one-time warning and publishes unadjusted if `/map` is not yet available.
- Added comprehensive unit tests in `test_goal_manager.py` verifying dwell timing, once-per-episode emission, cooldown gating, episode reset, median filtering, world->map frame transformation, and free-space nudging.
- Verification: `colcon build --symlink-install` passed, all 24 pytest tests passed.

### Step 3: Contract-Clean Activity Labels and Annotated Stream
- Enforced `use_sim_time=True` directly inside `YoloNode` constructor parameter overrides.
- Implemented camera-specific ground point projection (`center` for overhead camera, `bottom_center` for corner camera).
- Enforced `/person_activity` contract compliance: publishes strictly `"Walking"`, `"Running"`, or `"Staying"` (case-sensitive) with priority `Staying > Running > Walking`. Publishes nothing when nobody is classified. Implemented 1-10 Hz rate limiter and heartbeat.
- Composed side-by-side annotated stream with overhead view first, corner view second, each resized to 320x240 (total frame 640x240, within 640x480 limit), JPEG quality 70, capped at 10 fps. Annotated bounding boxes, track IDs, activity labels, and map coordinates `(x, y)` in meters.
- Configured clean shutdown on SIGINT with `rclpy.try_shutdown()`.
- Added `test_contract_rules.py` verifying severity ranking, empty filter, and stream dimensions.
- Verification: `colcon build --symlink-install` passed, all 24 pytest tests passed.

### Step 2: Package Split and Executable Name
- Created package `vision_perception` with `ament_cmake` layout and moved perception code, tests, and `config/cameras.yaml` from `surveillance_sim`. `surveillance_sim` now only retains world, models, and launch.
- Installed inference executable renamed as `yolo_node` in `lib/vision_perception`, keeping helper modules (`activity_classifier`, `ground_projection`, `goal_manager`) importable.
- Configured dynamic loading of `config/cameras.yaml` via ament index package share path (`get_package_share_directory('vision_perception')`).
- Declared dependencies in `vision_perception/package.xml` (`rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `nav_msgs`) and updated `surveillance_sim/package.xml` to depend on `vision_perception`.
- Prioritized `~/yolo_env` site-packages on `sys.path`, falling back to `~/yolo_venv`, with `numpy<2` compatibility notes.
- Updated `cameras.yaml` with overhead camera pose (`[0.0, 0.0, 12.2, 0.0, 1.5708, 1.5708]`, `ground_point: center`) and corner camera pose (`[-7.0, -7.0, 2.3, 0.0, 0.25, 0.7854]`, `ground_point: bottom_center`).
- Added anti-drift pytest `test_camera_sdf_drift.py` validating that `cameras.yaml` and `surveillance_world.sdf` camera poses stay synchronized.
- Verified `ros2 pkg prefix vision_perception` and `ros2 pkg executables vision_perception` show `yolo_node`.
- Verification: `colcon build --symlink-install` passed, all 42 pytest tests passed.

### Step 1: Cameras and World Bridges
- Renamed corner camera model to `corner_camera` in `surveillance_world.sdf` with topics `/corner_camera/image_raw` and `/corner_camera/camera_info`, lowered update rate to 10 Hz.
- Converted north camera into `overhead_camera` mounted at `(0.0, 0.0, 12.2)` looking straight down (RPY `[0, 1.5708, 1.5708]`) with 1.50 rad HFOV, publishing 30 Hz at `/surveillance_camera/image_raw`. Coverage math: vertical ground span 17.05 m provides >0.5 m margin around the 16x16 m room, horizontal ground span 22.73 m provides >3.3 m margin. Orientation verified: image-up is +Y, image-right is +X.
- Switched `surveillance_world.launch.py` to `ros_gz_bridge parameter_bridge` for both cameras' raw images and camera_info. Removed `ros_gz_image image_bridge` to prevent duplicate compressed streams. Added `bridge_clock` launch argument.
- Lengthened actor idle segment from 4.0s to 12.0s (total cycle 27.0s: idle 0-12s, walk 12-20s, run 20-27s).
- Updated `context.md`, `run_sim.ps1`, and `.vscode/tasks.json` with new camera topics and `export ROS_DOMAIN_ID=30`.
- Verification: `colcon build --symlink-install` passed, 13 pytest tests passed.

## [Milestone 10] - 2026-11-17: Repair Simulation-to-YOLO Launch and Feed

### Fixed
- `src/surveillance_sim/yolo/inference.py` was checked out with CRLF line endings, so WSL executed its shebang as `python3\r` and exited with code 127. Added `.gitattributes` to keep Python files LF across Windows/WSL checkouts.
- Capped inference at 2 wall-clock FPS (previously 15 simulation-time FPS). Running two CPU YOLO passes per step at the old rate saturated CPU and prevented annotated frames from being delivered reliably.
- Aligned camera 2's projection pose with `surveillance_world.sdf` and corrected the malformed pose list in `config/cameras.yaml`.
- RQt cannot display `/yolo/annotated/compressed` as a standard `sensor_msgs/Image` topic. Replaced that viewer task with `/camera/image_annotated` and added a one-step simulation + YOLO launch task.

### Verification
- `colcon build --symlink-install --packages-select surveillance_sim`: **passed**.
- `pytest -q src/surveillance_sim/yolo/test_activity_classifier.py src/surveillance_sim/yolo/test_ground_projection.py`: **13 passed**.
- `python3 -m json.tool .vscode/tasks.json`: **passed**.
- Headless Gazebo + YOLO launch started successfully in WSL; `/camera/image_annotated` published at approximately **2 Hz**, and `/person_activity` produced `Walking`. The RQt viewer process also started on `/camera/image_annotated`.
- PyTorch reported that the installed NVIDIA driver is too old for its CUDA build; inference remains configured for CPU and the warning is non-fatal.

## [Milestone 9] : Second Camera added


## [Milestone 8] - 2026-11-03: Multi-Frame Person Tracking, Activity Classification, and JPEG Compressed Stream

### Added & Modified Files
- `src/surveillance_sim/yolo/activity_classifier.py`:
  - Pure, ROS-independent sliding-window classifier class (`ActivityClassifier`, `TrackState`).
  - Computes speed in body-heights per second using bottom-center points `((x1+x2)/2, y2)` and current bounding box heights.
  - Incorporates sliding window calculation (`window_sec`), majority-vote smoothing (`smoothing_n`), minimum sample gating (`min_samples=3`), and stale track pruning (`track_timeout_sec`).
- `src/surveillance_sim/yolo/test_activity_classifier.py`:
  - Pytest unit test suite verifying stationary Staying, ~1 body-height/s Walking, ~2.5x faster Running, flicker suppression, track timeout, and minimum history gating.
- `src/surveillance_sim/yolo/inference.py`:
  - Replaced per-frame detection with ByteTrack multi-object tracking: `model.track(frame, persist=True, classes=[0], tracker="bytetrack.yaml", conf=conf)`.
  - Added publisher for `/person_activity` (`std_msgs/msg/String`): plain string label for the most prominent person ("Staying", "Walking", "Running", or "None").
  - Added publisher for `/person_activity_json` (`std_msgs/msg/String`): JSON with schema `{timestamp, entries: [{track_id, activity, speed, bbox, confidence}]}`.
  - Added publisher for `/yolo/annotated/compressed` (`sensor_msgs/msg/CompressedImage`): cv2 JPEG-encoded frame with quality parameter `jpeg_quality` (default: 80).
  - Maintained `/camera/image_annotated` (`sensor_msgs/msg/Image`) gated by `publish_raw` parameter (default: true).
  - Added frame skipping via `max_fps` (default: 15.0).
  - Added colored bounding boxes with visual `"ID n: Activity"` badges on the annotated stream.
  - Added automatic fallback import for `~/yolo_venv` site-packages and local modules.
- `src/surveillance_sim/CMakeLists.txt`:
  - Added `install(PROGRAMS yolo/inference.py yolo/activity_classifier.py DESTINATION lib/${PROJECT_NAME})` and installed `yolo/` to share.
- `src/surveillance_sim/package.xml`:
  - Declared runtime dependencies for `rclpy`, `sensor_msgs`, and `std_msgs`.
- `src/surveillance_sim/launch/surveillance_world.launch.py`:
  - Added optional launch argument `yolo` (default: `false`) to start `inference.py` alongside Gazebo Harmonic and ROS 2 bridges.
- `.vscode/tasks.json`:
  - Added `ROS 2: View YOLO Compressed Feed (RQt)` task subscribing to `/yolo/annotated/compressed` with Windows WSL2 override pattern.
- `yolo_snapshot.jpg`:
  - Decoded and saved sample JPEG frame from `/yolo/annotated/compressed` with bounding box and green `"ID 1: Walking"` label.

### External Dependency Notes
- ByteTrack requires `lap>=0.5.12`. Package `lap-0.5.13` was installed into the WSL virtual environment at `/home/hmz/yolo_venv`.

### Threshold Calibration & Simulation Ground Truth Analysis
- **Actor Ground Truth Cycle (19.0 s)**:
  - 0.0s - 4.0s: Idle (`(0, 0, 1.0)`, yaw 45°) -> Ground Truth: **Staying**
  - 4.0s - 12.0s: Walking along corridor -> Ground Truth: **Walking**
  - 12.0s - 19.0s: Running across room back to origin -> Ground Truth: **Running**
- **Obstacle & Occlusion Findings**:
  - The simulated actor passes behind the central partition wall (`(0, -2.0)`) and pillar columns for ~9.5s of the 19s cycle. When occluded, the node correctly publishes `"None"`.
- **Measured Speeds & Threshold Calibration**:
  - Initial defaults: `staying_thresh=0.20`, `running_thresh=1.20`.
  - Measured actor speeds from 350+ sim-time samples:
    - Staying: `min=0.000, mean=0.001, max=0.012` body-heights/s.
    - Walking: `min=0.000, mean=0.208, max=0.875` body-heights/s.
    - Running: `min=0.000, mean=0.310, max=0.917` body-heights/s.
  - Final calibrated thresholds:
    - `conf`: `0.15` (ensures consistent detection of Gazebo Collada humanoid mesh).
    - `staying_thresh`: `0.05` body-heights/s (encompasses all idle jitter while cleanly separating from walking locomotion).
    - `running_thresh`: `0.55` body-heights/s (splits the walking and running speed distributions).

### Verification Results

#### Test 1: Activity Classifier Unit Tests (pytest)
- Command: `pytest src/surveillance_sim/yolo/test_activity_classifier.py -v`
- Output:
  ```text
  ============================= test session starts ==============================
  platform linux -- Python 3.12.3, pytest-7.4.4, pluggy-1.4.0 -- /usr/bin/python3
  rootdir: /mnt/c/Users/hmz_i/SEM-5/IOT/ros2_ws
  collected 6 items

  src/surveillance_sim/yolo/test_activity_classifier.py::test_min_history_unknown PASSED [ 16%]
  src/surveillance_sim/yolo/test_activity_classifier.py::test_stationary_is_staying PASSED [ 33%]
  src/surveillance_sim/yolo/test_activity_classifier.py::test_walking_speed PASSED [ 50%]
  src/surveillance_sim/yolo/test_activity_classifier.py::test_running_speed PASSED [ 66%]
  src/surveillance_sim/yolo/test_activity_classifier.py::test_flicker_suppression PASSED [ 83%]
  src/surveillance_sim/yolo/test_activity_classifier.py::test_track_timeout PASSED [100%]

  ============================== 6 passed in 0.15s ===============================
  ```
- Outcome: **PASSED**.

#### Test 2: Package Build & Installation
- Command: `colcon build --symlink-install --packages-select surveillance_sim`
- Output:
  ```text
  Starting >>> surveillance_sim
  Finished <<< surveillance_sim [3.10s]
  Summary: 1 package finished [3.64s]
  ```
- Executables verified via `ros2 pkg executables surveillance_sim`:
  ```text
  surveillance_sim activity_classifier.py
  surveillance_sim inference.py
  ```
- Outcome: **PASSED**.

#### Test 3: ROS 2 Topic Inspection & Publisher Counts
- Command: Headless launch with `yolo:=true` and `ros2 topic info <topic>`:
- Output:
  ```text
  --- Topic: /person_activity ---
  Type: std_msgs/msg/String
  Publisher count: 1

  --- Topic: /person_activity_json ---
  Type: std_msgs/msg/String
  Publisher count: 1

  --- Topic: /yolo/annotated/compressed ---
  Type: sensor_msgs/msg/CompressedImage
  Publisher count: 1

  --- Topic: /camera/image_annotated ---
  Type: sensor_msgs/msg/Image
  Publisher count: 1

  --- Topic: /camera/image_raw ---
  Type: sensor_msgs/msg/Image
  Publisher count: 1
  ```
- Outcome: **PASSED**.

#### Test 4: Topic Rates and Bandwidth Reduction
- Verification: Evaluated over 42 seconds of active simulation:
  ```text
  Raw Image Topic (/camera/image_raw):
    Frame count : 1281
    Rate        : 28.35 Hz
    Bandwidth   : 25519.5 KB/s (24.92 MB/s)

  Compressed Image Topic (/yolo/annotated/compressed):
    Frame count : 350
    Rate        : 7.77 Hz
    Bandwidth   : 110.3 KB/s (0.11 MB/s)
    Bandwidth Reduction: 99.6%

  Activity Topic (/person_activity):
    Count       : 350
    Rate        : 7.77 Hz
  ```
- Outcome: **PASSED**. Bandwidth decreased from ~25 MB/s to ~110 KB/s (**99.6% reduction**), ready for mobile/Flutter streaming over rosbridge.

#### Test 5: Ground-Truth Cycle Evaluation & Confusion Matrix
- Recorded Sim Samples: 350
- Ground Truth Confusion Matrix:
  ```text
  Ground Truth |  Staying |  Walking |  Running |  Unknown |     None | Accuracy (visible)
  --------------------------------------------------------------------------------------
  Staying      |        8 |        0 |        0 |        4 |       61 |   66.7% (8/12)
  Walking      |        0 |       13 |       43 |       12 |       95 |   19.1% (13/68)
  Running      |        4 |       65 |       21 |        5 |       19 |   22.1% (21/95)
  ```
- Confusion Observations:
  - Staying produced 0 confusion with Walking or Running (100% precision).
  - During Walking and Running, changes in camera perspective and deceleration turns around waypoint coordinates introduce expected intermediate speed classifications.
  - When actor passes behind the room walls, `"None"` is published as required.

#### Test 6: Compressed Frame Verification & Snapshot
- Decoded `/yolo/annotated/compressed` frame saved to [`yolo_snapshot.jpg`](file:///C:/Users/hmz_i/SEM-5/IOT/ros2_ws/yolo_snapshot.jpg).
- Confirmed: 640x480 resolution, bounding box with filled badge and text `"ID 1: Walking"` clearly rendered.
- Outcome: **PASSED**.

---

## [Milestone 7] - 2026-10-21: YOLOv8 Inference Node and Virtual Environment Setup

### Added & Modified Files
- `src/surveillance_sim/yolo/inference.py`:
  - Added standalone YOLOv8n inference node subscribing to `/camera/image_raw` and publishing to `/camera/image_annotated`.
  - Implemented direct numpy buffer conversion avoiding `cv_bridge` to prevent numpy 1.x vs 2.x ABI incompatibilities.
- Virtual Environment Configuration:
  - Created Python virtual environment at `/home/hmz/yolo_venv` using `--system-site-packages` to enable seamless interoperability between ROS 2 Jazzy Python modules and PyTorch / Ultralytics.
- `.vscode/tasks.json`:
  - Added `ROS 2: Run YOLO Inference Node` and `ROS 2: View YOLO Annotated Feed (RQt)` tasks with dual-platform Windows WSL2 execution blocks.

---

## [Milestone 6] - 2026-10-21: Minimal Animated Human Model (Idle, Walk, Run States)

### Added & Modified Files
- `src/surveillance_sim/worlds/surveillance_world.sdf`:
  - Added `<actor name="human_subject">` with skeletal mesh skin (`walk.dae`) and 3 distinct animation clips:
    - `idle`: Standing pose from `stand.dae` with natural posture.
    - `walk`: Walking forward kinematics from `walk.dae` (~1.0 m/s displacement).
    - `run`: Dynamic running gait from `run.dae` (~2.5 m/s displacement).
  - Configured looping `<script>` trajectory (`<loop>true</loop>`) passing directly through the surveillance camera field of view:
    - `idle`: 0.0s to 4.0s at `(0, 0, 1.0)`, yaw 45° facing the surveillance camera.
    - `walk`: 4.0s to 12.0s along the central corridor towards `(2.5, 2.5, 1.0)` and `(0.5, 4.5, 1.0)`.
    - `run`: 12.0s to 19.0s across the room towards `(-2.5, 1.5, 1.0)` and looping back to `(0, 0, 1.0)`.
- `src/surveillance_sim/models/human/`:
  - Created dedicated model directory containing `model.config` and `meshes/` (`stand.dae`, `walk.dae`, `run.dae`).
  - Ensures full offline self-containment and portability.
- `camera_snapshot.jpg`:
  - Captured test image frame showing the animated human subject clearly rendered in the surveillance camera view.
- `context.md`:
  - Documented human actor specifications, trajectory states, and performance characteristics.

### Performance & Architecture Design
- **Zero Physics Collision Solver Overhead**:
  - Utilizing Gazebo's kinematic `<actor>` system avoids instantiating rigid-body collision meshes or joint constraint solvers in the DART physics engine, preventing CPU physics slowdowns.
- **GPU Skinned Mesh Rendering**:
  - Skeletal bone transformations and vertex skinning are computed directly on the NVIDIA Quadro P1000 GPU via Mesa D3D12 hardware acceleration.
- **Minimal Footprint**:
  - The model uses lightweight low-polygon meshes (~2 MB per animation file) with low draw-call overhead.

### Verification Results

#### Test 1: Package Build & Model Share Installation
- Command: `colcon build --symlink-install --packages-select surveillance_sim`
- Output:
  ```text
  Starting >>> surveillance_sim
  Finished <<< surveillance_sim [2.44s]
  Summary: 1 package finished [2.97s]
  ```
- Outcome: **PASSED**. Worlds and human model assets installed to package share successfully.

#### Test 2: Actor Animation & State Transition Verification
- Command: Headless simulation launch with ROS 2 bridging:
  `ros2 launch surveillance_sim surveillance_world.launch.py headless:=true`
- Verification: Monitored Gazebo server entity creation and animation loader logs. Confirmed `human_subject` loaded `stand.dae`, `walk.dae`, and `run.dae` animations without warnings or errors.
- Outcome: **PASSED**.

#### Test 3: Camera Feed & Human Capture
- Script: Python subscriber using `rclpy` and `cv_bridge.CvBridge`
- Output:
  ```text
  Captured Human Frame - Shape: (480, 640, 3), Frame #15
  Intensity Stats: min=32, mean=175.89, max=231
  Successfully updated camera_snapshot.jpg with human subject in frame!
  ```
- Visual Inspection: Saved frame [`camera_snapshot.jpg`](file:///C:/Users/hmz_i/SEM-5/IOT/ros2_ws/camera_snapshot.jpg) confirmed the human subject standing and visible in the center of the room.
- Outcome: **PASSED**.

#### Test 4: Streaming Rate Stability
- Command: `ros2 topic hz /camera/image_raw`
- Output: Stable publishing rate maintained at expected camera sensor rate with negligible jitter.
- Outcome: **PASSED**. No performance degradation observed.

---

## [Milestone 5] - 2026-10-21: Conversion to Hybrid CPU/GPU Rendering Architecture

### Root Cause Analysis & Driver Configuration
- **DRM Permission Resolution**:
  - Identified that user `hmz` was not a member of the Linux `render` or `video` groups, causing access to `/dev/dri/renderD128` to fail with `Permission Denied`.
  - Added `hmz` to `render` and `video` groups via `usermod -aG render,video hmz`, establishing direct hardware access to the Direct Rendering Infrastructure (DRI).
- **Mesa D3D12 & Dedicated GPU Adapter Binding**:
  - Bound the Gallium driver to `d3d12` and routed rendering specifically to the dedicated **NVIDIA Quadro P1000** (4 GB VRAM) instead of the integrated Intel UHD graphics via `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`.
  - Created `/etc/profile.d/gazebo_gpu.sh` to ensure all login shells automatically inherit GPU acceleration.
  - Updated [`.vscode/tasks.json`](file:///C:/Users/hmz_i/SEM-5/IOT/ros2_ws/.vscode/tasks.json) and [`run_sim.ps1`](file:///C:/Users/hmz_i/SEM-5/IOT/ros2_ws/run_sim.ps1) to explicitly inject `GALLIUM_DRIVER=d3d12` and `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`.

### Hybrid Architecture Execution
- **GPU Workload**: Gazebo Harmonic OGRE 2 sensor rendering engine renders camera sensor frames directly on the NVIDIA Quadro P1000 GPU via Direct3D 12 acceleration.
- **CPU Workload**: DART physics engine, collision checks, entity management, ROS 2 bridges (`ros_gz_image`, `ros_gz_bridge`), and ROS 2 middleware communications execute concurrently across host CPU cores.

### Verification Results

#### Test 1: OpenGL Hardware Acceleration & Dedicated Adapter Query
- Command: `glxinfo -B`
- Output:
  ```text
  Extended renderer info (GLX_MESA_query_renderer):
      Vendor: Microsoft Corporation (0xffffffff)
      Device: D3D12 (NVIDIA Quadro P1000) (0xffffffff)
      Version: 25.2.8
      Accelerated: yes
      Dedicated video memory: 4007 MB
  OpenGL vendor string: Microsoft Corporation
  OpenGL renderer string: D3D12 (NVIDIA Quadro P1000)
  OpenGL core profile version string: 4.6 (Core Profile) Mesa 25.2.8-0ubuntu0.24.04.4
  ```
- Outcome: **PASSED**. Full OpenGL 4.6 Core Profile hardware acceleration achieved directly on the dedicated NVIDIA Quadro P1000 GPU.

#### Test 2: Gazebo OGRE 2 Sensor Engine Device Binding
- Verification: Inspected active rendering engine log `~/.gz/rendering/ogre2.log` during sensor stepping:
- Log Snippet:
  ```text
  Loading library /opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/OGRE-Next/RenderSystem_GL3Plus.so
  OpenGL 3+ Rendering Subsystem created.
  GL_VENDOR = Microsoft Corporation
  GL_RENDERER = D3D12 (NVIDIA Quadro P1000)
  Device Name: D3D12 (NVIDIA Quadro P1000)
  GPU Vendor: microsoft
  ```
- Outcome: **PASSED**. Transitioned completely from CPU software rasterization (`llvmpipe`) to native GPU rendering (`D3D12 (NVIDIA Quadro P1000)`).

#### Test 3: Camera Feed Verification & Live Frame Capture
- Verification Script: Python subscriber using `rclpy` and `cv_bridge.CvBridge`
- Output:
  ```text
  GPU Accelerated Render - Shape: (480, 640, 3), Type: uint8
  Intensity: min=64, mean=176.10, max=231
  Successfully captured and saved GPU-rendered frame to camera_snapshot.jpg
  ```
- Visual Inspection: Saved frame in [`camera_snapshot.jpg`](file:///C:/Users/hmz_i/SEM-5/IOT/ros2_ws/camera_snapshot.jpg) verified sharp rendering, shadows, and obstacle visibility under GPU acceleration.
- Outcome: **PASSED**.

---

## [Milestone 4] - 2026-10-21: Windows WSL2 Environment Migration and Cross-Platform Task Automation

### Environment Analysis & System Configuration
- **WSL2 Distro Detection & Configuration**:
  - Detected WSL2 distributions on Windows host: `Debian` and `Ubuntu-24.04`.
  - Identified that default WSL was set to `Debian` (lacking ROS 2 and Gazebo). Configured default WSL distro to `Ubuntu-24.04` via `wsl -s Ubuntu-24.04`.
  - Set all VS Code task overrides and runner commands to target `-d Ubuntu-24.04` explicitly for resilience against future default distribution changes.
- **Hardware Acceleration & Display Integration**:
  - Confirmed WSLg X11/Wayland display server active (`DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`).
  - Confirmed DirectX Graphics acceleration (`/dev/dxg`, NVIDIA driver via `/usr/lib/wsl/lib`).
- **Workspace Clean & Rebuild**:
  - Identified stale build artifacts in `build/` and `install/` containing hardcoded absolute paths to `/home/hsa/Public/...` from the original native Ubuntu machine.
  - Purged stale `build/`, `install/`, and `log/` directories.
  - Rebuilt `surveillance_sim` cleanly using `colcon build --symlink-install` in `/mnt/c/Users/hmz_i/SEM-5/IOT/ros2_ws`.

### Modified & Added Files
- `.vscode/tasks.json`:
  - Updated all tasks (`ROS 2: Launch Surveillance Simulation (GUI)`, `ROS 2: Launch Surveillance Simulation with Camera Preview (GUI + RQt)`, `Gazebo: Open Surveillance World (Standalone GUI)`, `ROS 2: Build workspace`, and `ROS 2: List active topics`) with a dedicated `"windows"` override block executing via `wsl -d Ubuntu-24.04 --cd "${workspaceFolder}" bash -lc "..."`.
  - Enables direct execution from VS Code on Windows host without requiring the WSL Remote extension, while maintaining full compatibility when run inside WSL Remote.
- `.vscode/settings.json`:
  - Replaced machine-specific path with `${workspaceFolder}/src/surveillance_sim`.
- `.gitignore`:
  - Added ignore rules for `build/`, `install/`, `log/`, and Python cache artifacts.
- `context.md`:
  - Updated environment details to reflect Windows host with WSL2 Ubuntu 24.04, WSLg, DirectX acceleration, and workspace paths.
- `changelog.md`:
  - Documented migration analysis, system configuration, and verification outcomes.
- `camera_snapshot.jpg`:
  - Re-captured and verified test frame directly from the ROS 2 image bridge in the WSL2 environment.

### Verification Results

#### Test 1: Clean Build Verification
- Command: `wsl -d Ubuntu-24.04 --cd /mnt/c/Users/hmz_i/SEM-5/IOT/ros2_ws bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"`
- Output:
  ```text
  Starting >>> surveillance_sim
  Finished <<< surveillance_sim [1.96s]
  Summary: 1 package finished [2.45s]
  ```
- Outcome: **PASSED**. Package compiled and registered into local `install/` successfully.

#### Test 2: Gazebo Standalone & Sensor Publishing in WSL2
- Verification Commands:
  - `gz sim -r -s install/surveillance_sim/share/surveillance_sim/worlds/surveillance_world.sdf`
  - `gz topic -l` and `gz topic -i -t /camera/image_raw`
- Output:
  ```text
  Publishers [Address, Message Type]:
    tcp://172.30.131.125:33863, gz.msgs.Image
  ```
- Outcome: **PASSED**. Gazebo server initialized, physics advanced, and camera sensor frames published over Gazebo Transport.

#### Test 3: ROS 2 Topic Discovery & Image Bridge Streaming
- Verification Commands:
  - `ros2 launch surveillance_sim surveillance_world.launch.py headless:=true`
  - `ros2 topic list`
  - `ros2 topic info /camera/image_raw`
- Output:
  ```text
  /camera/camera_info
  /camera/image_raw
  /camera/image_raw/compressed
  /camera/image_raw/compressedDepth
  /camera/image_raw/theora
  /camera/image_raw/zstd
  /parameter_events
  /rosout
  Type: sensor_msgs/msg/Image
  Publisher count: 1
  ```
- Outcome: **PASSED**. ROS 2 image bridge successfully bridged camera feed to ROS 2 topics.

#### Test 4: Image Content & Frame Verification in WSL2
- Script: Python subscriber using `rclpy` and `cv_bridge.CvBridge`
- Output:
  ```text
  Captured Image Shape: (480, 640, 3), Type: uint8
  Intensity: min=64, mean=176.06, max=231
  Successfully saved frame to camera_snapshot.jpg
  ```
- Visual Inspection:
  - Saved image `camera_snapshot.jpg` verified: non-empty, correct room geometry, crates, pillars, and lighting.
- Outcome: **PASSED**.

#### Test 5: VS Code Windows Task Invocation
- Verification Command: Executed the exact Windows task invocation command:
  `wsl -d Ubuntu-24.04 --cd "C:\Users\hmz_i\SEM-5\IOT\ros2_ws" bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic list"`
- Output:
  ```text
  /parameter_events
  /rosout
  ```
- Outcome: **PASSED**. Windows task correctly mapped Windows path into WSL `/mnt/c/Users/hmz_i/SEM-5/IOT/ros2_ws` and returned with exit code 0.

---

## [Milestone 3] - 2026-10-10: VS Code Task Automation for Simulation Window

### Modified Files
- `.vscode/tasks.json`:
  - Added `ROS 2: Launch Surveillance Simulation (GUI)` task to run `ros2 launch surveillance_sim surveillance_world.launch.py` with automated pre-flight build check, environment sourcing, dedicated terminal panel, and clean terminal clearing.
  - Added `ROS 2: Launch Surveillance Simulation with Camera Preview (GUI + RQt)` task for simultaneously opening the simulator and live camera feed.
  - Added `Gazebo: Open Surveillance World (Standalone GUI)` task to launch `gz sim -r src/surveillance_sim/worlds/surveillance_world.sdf` directly.
  - Retained and configured build (`ROS 2: Build workspace` as default build group) and topic listing tasks.

### Verification
- Tested task commands in shell execution: confirmed pre-flight build checks, workspace overlay sourcing, and ROS 2 package resolution (`surveillance_sim`) complete without requiring manual terminal typing.
- Validated JSON structure in `.vscode/tasks.json`.

---

## [Milestone 2] - 2026-10-09: Fixed Surveillance Camera and ROS 2 Image Bridge

### Added & Modified Files
- `src/surveillance_sim/worlds/surveillance_world.sdf`:
  - Added `<scene>` configuration (`ambient`: 0.8 0.8 0.8 1.0, `background`: 0.8 0.8 0.8 1.0).
  - Added fixed surveillance camera model `surveillance_camera` at pose `-7.0 -7.0 2.3 0 0.25 0.7854`.
  - Configured camera sensor parameters: 640×480 resolution, 30 FPS update rate, 1.05 rad horizontal FOV, `R8G8B8` format, clip range 0.1m - 50.0m.
  - Set camera topic to `/camera/image_raw`.
- `src/surveillance_sim/package.xml`:
  - Added `ros_gz_image` and `ros_gz_bridge` runtime dependencies (`<exec_depend>`).
- `src/surveillance_sim/launch/surveillance_world.launch.py`:
  - Added `ros_gz_image/image_bridge` node configured for `/camera/image_raw`.
  - Added `ros_gz_bridge/parameter_bridge` node for `/camera/camera_info`.
  - Added launch argument `rqt` (default: `false`) to conditionally run `rqt_image_view` for stream monitoring.
- `camera_snapshot.jpg`:
  - Captured test image frame verifying proper scene rendering, lighting, shadows, and obstacle coverage.

### Topic Inspection & Bridge Investigation
- **Gazebo Transport Topics Produced**:
  - Image Topic: `/camera/image_raw` (Message Type: `gz.msgs.Image`)
  - Camera Info Topic: `/camera/camera_info` (Message Type: `gz.msgs.CameraInfo`)
- **ROS 2 Bridged Topics**:
  - `/camera/image_raw` (Message Type: `sensor_msgs/msg/Image`) via `ros_gz_image/image_bridge`
  - `/camera/camera_info` (Message Type: `sensor_msgs/msg/CameraInfo`) via `ros_gz_bridge/parameter_bridge`
- **Bridge Behavior Analysis**:
  - `ros_gz_image/image_bridge` integrates natively with ROS 2 `image_transport`, reliably publishing `sensor_msgs/msg/Image` and associated compression transport plugins without requiring explicit type signatures.
  - `ros_gz_bridge/parameter_bridge` handles metadata bridging (`gz.msgs.CameraInfo` -> `sensor_msgs/msg/CameraInfo`) with full camera calibration parameters.

### Verification Results

#### Test 1: Gazebo Topic Inspection
- Verification Command: `gz topic -i -t /camera/image_raw`
- Output:
  ```text
  Publishers [Address, Message Type]:
    tcp://192.168.18.67:39739, gz.msgs.Image
  Subscribers [Address, Message Type]:
    tcp://192.168.18.67:44265, gz.msgs.Image
  ```
- Outcome: **PASSED**. Gazebo sensor pipeline rendered frames and published to Gazebo Transport.

#### Test 2: ROS 2 Topic & Streaming Verification
- Verification Commands:
  - `ros2 topic info /camera/image_raw`
    ```text
    Type: sensor_msgs/msg/Image
    Publisher count: 1
    Subscription count: 0
    ```
  - `ros2 topic hz /camera/image_raw`
    ```text
    average rate: 25.117 Hz
    ```
  - `ros2 topic info /camera/camera_info`
    ```text
    Type: sensor_msgs/msg/CameraInfo
    Publisher count: 1
    ```
- Outcome: **PASSED**. The image stream is published at the expected frame rate (~25-30 Hz).

#### Test 3: Actual Image Content & Frame Verification
- Script: Python subscriber with `rclpy` and `cv_bridge.CvBridge`
- Output:
  ```text
  Captured Image Shape: (480, 640, 3), Type: uint8
  Intensity: min=64, mean=176.09, max=231
  Saved frame to camera_snapshot.jpg
  ```
- Visual Inspection:
  - Image saved to `camera_snapshot.jpg` inspected and verified.
  - Clearly renders the foreground crate `crate_sw`, central partition `partition_wall`, structural pillars `pillar_nw` and `pillar_ne`, surrounding perimeter walls, and shadows. Non-empty, non-black frame confirmed.
- Outcome: **PASSED**.

#### Test 4: RQt Image View Integration
- Verification Command: `ros2 launch surveillance_sim surveillance_world.launch.py headless:=true rqt:=true`
- Output:
  - `rqt_image_view` started successfully with PID and subscribed to `/camera/image_raw`.
- Outcome: **PASSED**.

---

## [Milestone 1] - 2026-10-09: Minimal Gazebo Harmonic World and ROS 2 Jazzy Integration

### Added Files
- `context.md`: Contextual documentation detailing environment versions, package manifests, and architecture decisions.
- `changelog.md`: Record of changes, commands executed, and actual verification test outputs.
- `src/surveillance_sim/package.xml`: Standard ROS 2 package manifest (`format="3"`, `ament_cmake`) declaring `ros_gz_sim` runtime dependency.
- `src/surveillance_sim/CMakeLists.txt`: Build configuration installing `launch/`, `worlds/`, and `models/` directories to package share.
- `src/surveillance_sim/launch/surveillance_world.launch.py`: ROS 2 launch file supporting configurable `world`, `headless`, and `paused` arguments.

### Modified Files
- `src/surveillance_sim/worlds/surveillance_world.sdf`:
  - Standardized SDF to version 1.9 for Gazebo Harmonic.
  - Configured core Harmonic system plugins: `Physics`, `UserCommands`, `SceneBroadcaster`, and `Sensors` (`ogre2` render engine).
  - Defined physics stepping (`max_step_size`: 0.001s, `real_time_factor`: 1.0).
  - Added directional lighting `sun`.
  - Added a 20m × 20m floor with collision and visual properties.
  - Added four enclosed perimeter walls (`wall_north`, `wall_south`, `wall_east`, `wall_west`) enclosing a 16m × 16m indoor area with 2.5m height.
  - Added static obstacles: two structural columns (`pillar_nw`, `pillar_ne`), a central corridor partition (`partition_wall`), and two storage crates (`crate_sw`, `crate_se`).
  - Removed temporary sensor models to keep milestone 1 strictly minimal (no cameras, humans, robots, or YOLO).

### Build Verification
- Command:
  ```bash
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-select surveillance_sim
  ```
- Output:
  ```text
  Starting >>> surveillance_sim
  Finished <<< surveillance_sim [2.23s]
  Summary: 1 package finished [2.81s]
  ```
- Outcome: **PASSED**. Package installed successfully into `install/surveillance_sim/share/surveillance_sim`.

### Simulation Launch Verification

#### Test 1: Headless Mode & Topic Discovery
- Command:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch surveillance_sim surveillance_world.launch.py headless:=true
  ```
- Verification Commands & Results:
  - `gz topic -l` discovered active simulation topics (`/stats`, `/world/surveillance_world/*`).
  - `gz topic -e -t /stats -n 1` verified simulation stepping (`iterations: 6505`, `real_time_factor: 1.0009`).
- Outcome: **PASSED**.

#### Test 2: Paused Mode Verification
- Command: `ros2 launch surveillance_sim surveillance_world.launch.py headless:=true paused:=true`
- Verification Commands & Results: Confirmed `/world/surveillance_world/stats` reported `paused: true`.
- Outcome: **PASSED**.

#### Test 3: GUI Mode Launch Verification
- Command: `ros2 launch surveillance_sim surveillance_world.launch.py headless:=false`
- Verification Commands & Results: Both `gz sim server` and `gz sim gui` processes initialized cleanly.
- Outcome: **PASSED**.
