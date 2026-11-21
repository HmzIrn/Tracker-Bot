# Integration Measurements and History

[`../INTERFACES.md`](../INTERFACES.md) is the stable interface contract. This file records experiments, observed values, and unresolved measurement questions; it is not a second source of topic defaults or setup instructions.

## A2-A4: Detection, projection, and classifier experiments

### A2 overhead/corner detection experiment

An actor-visible native frame pair was captured at sim stamp 3.0 s and later deleted during C1 cleanup. Approximate actor extent was **15x16 px overhead** and **14x80 px corner** (partially occluded). YOLOv8n/s/m ran on CPU at `conf=0.15`:

| Model | Overhead detections, imgsz 640 / 960 / 1280 | Corner detections, imgsz 640 / 960 / 1280 |
| :--- | :--- | :--- |
| YOLOv8n | 0 / 0 / 0 | 0 / 1 (`0.211`, 13.1x55.0 px) / 1 (`0.419`, 14.0x56.3 px) |
| YOLOv8s | 0 / 0 / 0 | 0 / 1 (`0.228`, 12.9x55.8 px) / 2 (`0.471`, 13.5x55.8 px; `0.334`, 13.0x80.6 px) |
| YOLOv8m | 0 / 0 / 0 | 0 / 0 / 1 (`0.399`, 14.1x54.6 px) |

The headless run reached **42.87 simulated seconds** before its 110-second wall guard, rather than the requested 60 seconds. No camera resolution or model default was changed. The result supports relying on projected corner-track markers in the overhead annotation rather than overhead detections.

### A3 projection calibration

The SDF/YAML drift test compares both camera poses, HFOV (1.50/1.05 rad), and 640x480 resolution. Integer-pixel geometric round-trip error was **0.00004 m overhead at room center** and **0.06817 m corner near the wall**. These are pixel-quantization errors, not detector localization accuracy.

### A4 classifier sampling

At `max_fps=2.0`, `window_sec=1.0`, 42.07 sim seconds yielded 33 tracked updates across tracks 1-6. Sample counts per 1-second window had min/median/max **1/3/4**, and **11/33** updates had fewer than `min_samples=3`. Replaying those positions with `window_sec=2.0` gave **1/5/6** samples and **9/33** updates below three.

Confusion matrices (rows are ground truth Idle / Walk / Run; columns are predicted Staying / Walking / Running / Unknown):

| Window | Idle | Walk | Run |
| :--- | :--- | :--- | :--- |
| 1.0 s | 0 / 0 / 0 / 2 | 0 / 8 / 0 / 5 | 0 / 15 / 1 / 2 |
| 2.0 s replay | 0 / 0 / 0 / 2 | 0 / 8 / 0 / 5 | 0 / 16 / 0 / 2 |

The 2-second replay reduced sample starvation but did not improve the confusion matrix. The window was set to 2.0 s; speed thresholds were not changed. The CPU-limited run did not justify increasing inference frequency.

## F1-F3: Goal, actor speed, and corner recall

### F1 end-to-end goals

With the transient-local all-free test map (origin `(-5,-5)`, resolution `0.1 m`, size `100x100`), the earlier 55-sim-second run observed exactly one valid goal in each of two idle episodes. Both were within **0.14 m** of `(0,0)` and had `frame_id=map`, `z=0`, `w=1`. A separate **43 sim-second** run without the test-map helper produced **zero goals** and throttled missing-map warnings.

### F2 waypoint speed comparison

Current SDF waypoint segment speeds are:

| Segment | Speed (m/s) | Comparison with Staying `<0.3` and Running `>=1.75` thresholds |
| :--- | ---: | :--- |
| Idle | 0 | Staying |
| Walk segments | 0.875, 1.100, then 1.000 m/s segments | Above Staying, below Running |
| Run segments | 3.000, 3.138 | Above Running |

An earlier trajectory measurement (before the H2 route update) reported walk/run segment speeds of 1.010/1.131 and 1.697/1.166 m/s; those values are historical and superseded by the current SDF waypoint speeds. No speed threshold or route change was made for this comparison.

### F3 corner-camera recall

Across **55 unoccluded actor-visible frames** from two cycles, each model/size combination detected the actor in **55/55** frames. Mean actor pixel height was **80.95 px** (range 63-94).

| Model / imgsz | Mean / median CPU time per frame |
| :--- | :--- |
| YOLOv8n / 640 | 115.3 / 58.6 ms |
| YOLOv8s / 640 | 156.1 / 151.5 ms |
| YOLOv8n / 960 | 143.0 / 133.9 ms |
| YOLOv8s / 960 | 354.9 / 350.3 ms |

Recommendation from that sample: retain v8n at 640 pending broader data. Unlike the partly occluded A2 sample, the current world has no central partition.

## F4: Camera-rate experiment

At the configured 30 Hz overhead source rate, measured RTF was **0.763**. A temporary 15 Hz SDF copy measured **0.826** (+8.3%). Sim-time inference/annotation rates shifted from about **2.47 to 2.22 Hz**, while wall rates remained about **1.95/1.91 Hz**. The source SDF was not changed; ask before making any camera-rate change permanent.

## H4: Three-cycle runtime verification

The headless run used `bridge_clock:=true`, the transient-local test map, and bare `ros2 run vision_perception yolo_node`. It collected **81.016 sim seconds / 120.190 wall seconds (RTF 0.674)**; `/clock` and image-header spans differed by **0.063 s**.

Three valid goals were emitted, one during each idle interval:

| Image-stamp time | Map position `(x,y)` m |
| ---: | :--- |
| 11.683 s | `(-0.118,-0.075)` |
| 35.377 s | `(-0.109,-0.070)` |
| 61.678 s | `(-0.109,-0.064)` |

All were within 0.14 m of the idle point `(0,0)` and had `frame_id=map`, `z=0`, `w=1`. The no-map result is recorded under F1.

Classification counts assign each inference sample to the closest tracked position to the SDF-interpolated actor position; columns are Staying / Walking / Running / Unknown:

| Ground-truth segment | Staying | Walking | Running | Unknown |
| :--- | ---: | ---: | ---: | ---: |
| Idle | 67 | 15 | 0 | 2 |
| Walk in | 14 | 18 | 0 | 0 |
| Run | 0 | 15 | 4 | 6 |
| Walk return | 0 | 58 | 9 | 0 |

Corner detections occurred in **208/210** inference samples (99.0%); two idle samples had no tracked entry. The nearest-actor track ID changed five times across the run, and nine inference samples contained multiple IDs.

| Measurement | Messages | Wall Hz | Sim Hz |
| :--- | ---: | ---: | ---: |
| Overhead camera input | 1390 | 11.557 | 17.145 |
| Corner camera input | 436 | 3.624 | 5.377 |
| Inference telemetry | 210 | 1.874 | 2.588 |
| `/person_activity` | 100 | **0.902** | **1.335** |
| Overhead annotated JPEG | 209 | 1.872 | 2.754 |
| Corner annotated JPEG | 210 | 1.874 | 2.590 |

**The 0.902 Hz wall-clock `/person_activity` result was measured before the C1b fix.** All 100 strings were valid (41 Staying, 51 Walking, 8 Running); the annotated images were 640x480 JPEG at configured quality 75. Sampled run frames showed visible foot glide during the approximately 3.0-3.14 m/s run. This was a qualitative check; no animation or route change was made.

## C1-C1b: Cleanup and wall-clock activity rate

C1 removed four saved image frames and three workspace YOLO weights; the node uses the bare `yolov8n.pt` model name, not a workspace-local weights path. Python and pytest caches were removed. The repository spelling check for the former typo returned zero hits; registration is named `Surveillance_Sim` and uses `~/.surveillance_sim.bash`.

C1b added `activity_publish_hz` (default 2.0, clamped to 1.5-10.0), immediate label changes, wall-clock repeats, silence for invalid/no labels, and a 3-second inference-staleness cutoff. Five fake-wall-clock tests cover change, cadence, clamping, silence, and staleness.

The single requested headless run recorded **18.052 sim seconds / 22.501 wall seconds (RTF 0.802)**:

| Measurement | Wall Hz | Sim Hz |
| :--- | ---: | ---: |
| Overhead camera input | 17.51 | 21.83 |
| Corner camera input | 6.00 | 7.49 |
| Inference telemetry | 1.97 | 1.92 |
| `/person_activity` | **1.749** | **2.142** |
| Overhead annotated JPEG | 1.943 | 2.410 |
| Corner annotated JPEG | 1.972 | 1.932 |

The run observed 28 valid activity messages and one `/goal_pose` at image stamp **10.792 s** in the idle interval. The 1.749 Hz measurement used the initial executor-timer implementation; its maximum message gap was **0.893 s**. Publishing was then moved to a dedicated wall-clock worker thread to prevent inference callback starvation. D1 later measured that final worker: the all-message wall average was **1.615 Hz**, with a **21.952 s** maximum overall gap. The active-only rate and interval could not be separated from periods with no valid classification; see D1 below. Do not treat the initial 1.749 Hz result as a measurement of the final thread.

Ultralytics re-created `yolov8n.pt` in the workspace root during this run; final cleanup removed it again along with regenerated caches.

The requested `ros2 topic info -v` and `ros2 topic hz` queries occurred during teardown after perception publishers had disappeared. They do not establish live perception endpoint QoS or rates. The measurements above are recorder rates; source-derived node QoS and the test-map QoS are documented in [`../INTERFACES.md`](../INTERFACES.md).

## D1: Live integration verification

The single D1 headless run used `bridge_clock:=true`, the transient-local test map, and the bare `ros2 run vision_perception yolo_node` command. It reached RTF **0.63265** and recorded **78,045** `/clock` callbacks, **1,627/498** overhead/corner images, **2,765/912** overhead/corner `CameraInfo` messages, **203** telemetry messages, **201/203** overhead/corner annotations, **199** activity messages, and **4** goals. These are observer counts, not rates: the short `ros2 topic hz` captures did not return stable rates for the camera, annotation, activity, or goal topics. `/clock` produced one live CLI sample of **539.3 Hz**. Do not infer zero publication from the failed rate captures.

All **199** activity strings were valid (`Walking` 109, `Staying` 86, `Running` 4). The observed average between activity messages was **1.615 Hz wall-clock**; the maximum gap across all messages was **21.952 s**. The output did not retain enough per-track classification timing to determine whether that gap occurred while a valid label was continuously classified, so active-only cadence and maximum interval remain **unverified**. D1's message counts and average therefore do not prove the configured active-label interval contract.

Four goals were recorded in idle segments. Their image stamps and map positions were `(11.452 s, -0.1143, -0.0715)`, `(34.882 s, -0.1096, -0.0746)`, `(61.381 s, -0.1108, -0.0726)`, and `(88.969 s, 0.8951, -0.0656)`. Each had `frame_id=map`, `z=0`, and `w=1`. The first three were near the default idle point `(0,0)`. Setting the live `world_to_map` parameter to `[1,0,0]` succeeded but did not change the already-constructed `GoalPoseManager`; editing `cameras.yaml` to `[1,0,0]` and restarting without a rebuild changed the next goal's x coordinate by about 1 m. The source YAML was restored to `[0,0,0]` after the run. Runtime parameter updates to `world_to_map` are therefore not effective for goal generation; use the YAML default at node startup.

Live `ros2 topic info -v` captures reported reliable/volatile endpoints for the observed camera bridge and perception publishers; ROS CLI reported history depth as `UNKNOWN`. The first overhead raw-image query happened before its publisher was discovered, so that endpoint's QoS was not directly verified in the capture. The `/clock` publisher was reliable/volatile and the `use_sim_time` subscriber best-effort/volatile. `/surveillance_camera/image_raw/compressed` was discovered as `sensor_msgs/msg/CompressedImage` with a publisher, but no message/rate was confirmed. `compressed_image_transport` and `image_transport` were installed. Other short `ros2 topic hz` commands ended without a usable rate; their SIGTERM teardown emitted invalid-context diagnostics.

The YOLO process returned from SIGINT in **3.68 s** and **2.06 s** on its two starts, within the six-second limit and without SIGKILL. Startup to the initialized log took **20.23 s**; the D1 temporary working directory contained a **6,549,796-byte** `yolov8n.pt`, but the harness did not isolate download time. Its observer reported first annotation **47.00 s** after initialization, inconsistent with the earlier goal/inference evidence; callback-queue delay made this latency unreliable. First-inference latency and download-only time are **unverified**. The harness's end-of-run process check found one `gz sim` child (PID 590); a later WSL process check found no such process, but orderly Gazebo-child cleanup was not confirmed during the run.

The workspace-root `yolov8n.pt` present before D1 was preserved: it was not created by this task and the current task permits deleting only task-created temporary files. It is local and untracked; the node still passes the bare model name to Ultralytics.

## Remaining limitations and open decisions

- The overhead detector failed on all A2 sample frames; its actor was only about 15x16 px. Corner-derived global markers supply overhead annotations.
- Running recognition remains weak on the H4 route, and track IDs fragmented during the run. No threshold, smoothing, or route tuning is proposed by these measurements.
- D1 measured a 1.615 Hz wall-clock average between activity messages, but the **21.952 s** maximum overall gap cannot be classified as an active-label gap from retained data. Measure the active-only interval on a future run before claiming that the repeat requirement is empirically verified.
- CPU RTF remained below 1.0. PyTorch was built for CUDA 13.0 while the available WSL driver capability was 12.5; `torch.cuda.is_available()` was false. No GPU software was installed. A supported driver/runtime and GPU pairing would be needed to revisit GPU inference.
- One earlier Gazebo SIGINT teardown reported a segmentation fault after measurement output had been produced.
- The goal free-cell rule is `cell == 0`, and map-origin yaw is assumed zero. Map production and `world_to_map` calibration belong to the consuming integration.
- External Gazebo Fuel animation assets may require network access or a populated cache on first launch.
- D1's camera/annotation/activity/goal `ros2 topic hz` captures were inconclusive; the observer retained counts but not reliable per-topic timing for those feeds. The image-compress republisher's output type was discovered, but its messages and rate were not.

## Verification history

| Checkpoint | Build | Tests |
| :--- | :--- | :--- |
| C1 cleanup | `colcon build --symlink-install` passed | `pytest -q`: 68 passed |
| C1b activity publisher | `colcon build --symlink-install` passed | Focused activity tests: 5 passed; full `pytest -q`: 78 passed |
| D1 live verification | `colcon build --symlink-install` passed | `pytest -q`: 78 passed |
