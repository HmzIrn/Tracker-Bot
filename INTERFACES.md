# Surveillance_Sim Integration Guide

| Contract field | Value |
| :--- | :--- |
| Contract version | 1.0 |
| Date | 2026-11-21 |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| ROS domain | `ROS_DOMAIN_ID=30` |

This is the stable integration reference. [docs/INTEGRATION_NOTES.md](docs/INTEGRATION_NOTES.md) holds measurements, experiment history, and unresolved runtime observations.

Evidence labels: **[code]** means source/config/SDF; **[verified]** means observed in a recorded check; **[per Flutter contract]** is an external app requirement; **[unverified, owner]** needs confirmation from the named integration owner. D1 used the one allowed live simulation; no second simulation is implied below.

## Quick reference

1. **Project:** `Surveillance_Sim`, ROS 2 Jazzy, Gazebo Harmonic, domain 30. [code]
2. **Packages:** `surveillance_sim` provides the world/cameras; `vision_perception` provides `yolo_node`. [code]
3. **Register once:** `bash scripts/register_with_ros.sh`; VS Code task `Surveillance_Sim: Register with ROS 2 (run once)`. [code]
4. **Non-interactive shell:** `source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30`. [code]
5. **World-only launch:** `ros2 launch surveillance_sim surveillance_world.launch.py headless:=true bridge_clock:=true`. [verified]
6. **Perception process:** `ros2 run vision_perception yolo_node` with no arguments or stdin. [verified]
7. **Activity:** `/person_activity`, `std_msgs/msg/String`, exact labels `Walking`, `Running`, `Staying`; wall-clock repeats, default 2 Hz. [code]
8. **Annotated feeds:** `/yolo/annotated/compressed` (overhead) and `/yolo/corner/annotated/compressed` (corner), native 640x480 JPEG. [code]
9. **Goals:** `/goal_pose`, `geometry_msgs/msg/PoseStamped`, map frame; requires a transient-local `/map`. [code]
10. **App camera:** Camera tab consumes `/surveillance_camera/image_raw/compressed`; app ROS Bridge URL is `ws://localhost:9090`. [per Flutter contract]

## Contents

1. [Ownership and deployment](#ownership-and-deployment)
2. [Quick start and prerequisites](#quick-start-and-prerequisites)
3. [Processes and launch arguments](#processes-and-launch-arguments)
4. [Published topics](#published-topics)
5. [Consumed topics](#consumed-topics)
6. [Message semantics](#message-semantics)
7. [World, coordinates, and cameras](#world-coordinates-and-cameras)
8. [Parameters](#parameters)
9. [Not provided](#not-provided)
10. [Integration runbook and checklists](#integration-runbook-and-checklists)
11. [Known limitations and open decisions](#known-limitations-and-open-decisions)
12. [Change control](#change-control)

## Ownership and deployment

| Owner | Owns |
| :--- | :--- |
| This project | `surveillance_sim`: Gazebo world, actor, sensors, camera and optional clock bridges. `vision_perception`: two-camera inference, projection/fusion, tracking, activity classification, annotations, and `/goal_pose`. [code] |
| Robot/navigation teammate | Robot model and bridges, spawn, TF, `/map`, mapping, localization, Nav2, `/odom`, `/scan`, and `/cmd_vel`. This workspace does not supply these. [code] |
| Flutter app owner | App Launcher configuration, ROS Bridge connection, app subscriptions, mock location/navigation behavior, and process signal handling. [per Flutter contract] |

Use one workspace-overlay strategy at a time:

- **Option A - app workspace (recommended for the app PC):** place the `surveillance_sim` and `vision_perception` package directories directly under `~/wahab_ros_work/src/`, then build that workspace. Do not copy the repository as a nested `src/ros2_ws/src/...` tree. Use the app workspace overlay only. [per Flutter contract]
- **Option B - this standalone workspace:** build/register this checkout with `bash scripts/register_with_ros.sh`; the generated `~/.surveillance_sim.bash` sources ROS Jazzy and this workspace's `install/setup.bash`. The app's non-interactive launcher must resolve these packages from this overlay. Do not also source a conflicting `wahab_ros_work` overlay; remove this registration with `bash scripts/register_with_ros.sh --remove` before switching strategies. [code]

The script does not set `ROS_DOMAIN_ID`; every participating process must be started in domain 30. [code]

## Quick start and prerequisites

Run ROS, `colcon`, and pytest in WSL Ubuntu 24.04 from the workspace root. Windows PowerShell is not the ROS build/runtime shell. [verified]

Register after cloning or moving the workspace:

```bash
bash scripts/register_with_ros.sh
```

The matching VS Code task is `Surveillance_Sim: Register with ROS 2 (run once)`. `--no-build` validates and registers an already built workspace; `--remove` removes the marked `.bashrc` block and generated registration file. The script checks package prefixes, the `yolo_node` executable, and launch argument resolution. [code]

Minimum runtime dependencies are ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, `colcon`, NumPy `<2`, OpenCV, PyYAML, PyTorch, and Ultralytics. The node prepends `~/yolo_env` to its Python import path (then checks `~/yolo_venv`). [code]

The node constructs `YOLO("yolov8n.pt")`; weights are not installed by this repository. Ensure that Ultralytics can resolve the model, or allow it to download on first use. D1 created a 6,549,796-byte model in its temporary process working directory; download-only time and a persistent cache location were not measured. A pre-existing ignored workspace-root weight was preserved and is not required by a repository-local path. [code, verified; cache details unverified, perception owner]

The actor's skin and idle/walk/run DAE animation files use Gazebo Fuel URLs. A Fuel network connection or populated Gazebo cache may be needed on first world launch. [code]

Start the world and node in separate WSL shells:

```bash
source ~/.surveillance_sim.bash
export ROS_DOMAIN_ID=30
ros2 launch surveillance_sim surveillance_world.launch.py headless:=true bridge_clock:=true
```

```bash
source ~/.surveillance_sim.bash
export ROS_DOMAIN_ID=30
ros2 run vision_perception yolo_node
```

The app starts `yolo_node` with the bare command, no arguments and no stdin. Model loading is synchronous before camera subscriptions and output publishers exist. D1 measured **20.23 s** from process start to the initialized log; this includes model resolution/loading. First-inference latency is unverified. Stop with SIGINT; if still running after 6 seconds, send SIGKILL. D1's two YOLO processes exited in 3.68 s and 2.06 s with SIGINT. [code, verified]

## Processes and launch arguments

| Process | Started by / command | Publishes and subscribes | Shutdown |
| :--- | :--- | :--- | :--- |
| Gazebo and camera bridge | `ros2 launch surveillance_sim surveillance_world.launch.py ...` | Starts Gazebo with the configured SDF and `ros_gz_bridge parameter_bridge`; publishes two camera images and two CameraInfo topics. | SIGINT to the launch process. D1's end-of-run check saw a `gz sim` child after launch teardown; a later shell found no such process. Orderly child cleanup remains unverified. |
| Clock bridge | World launch when `bridge_clock:=true` | Publishes `/clock` from Gazebo. | SIGINT with world launch. |
| `rqt_image_view` | World launch when `rqt:=true` | Subscribes to `/surveillance_camera/image_raw`. | SIGINT with world launch. |
| `yolo_node` | World launch when `yolo:=true`, or bare `ros2 run vision_perception yolo_node` | Subscribes to both camera images and `/map`; publishes perception outputs. Uses ROS sim time, with image stamps driving classifier and dwell timing. | SIGINT; SIGKILL only after 6 s if needed. |
| Test-map helper | `python3 src/vision_perception/test_tools/publish_test_map.py` | Publishes an all-free transient-local `/map`; non-installed test helper, not a production map provider. | SIGINT. |
| Camera Compress republisher | App-owned image-transport card; see [Flutter runbook](#37-flutter-launcher-and-topic-selection). | Intended to republish overhead raw input as `/surveillance_camera/image_raw/compressed`. D1 found a publisher of the expected type but did not confirm messages or rate. | Stop the card's process using its configured signal handling; exact app behavior is unverified, Flutter owner. |

| Launch argument | Default | Effect |
| :--- | :--- | :--- |
| `world` | Package-share `surveillance_sim/worlds/surveillance_world.sdf` | SDF file passed to Gazebo. |
| `headless` | `false` | `true` adds server-only `-s`; `false` allows the GUI. |
| `paused` | `false` | `false` adds `-r` to run on start; `true` omits it. |
| `bridge_clock` | `false` | Starts the ROS `/clock` bridge when true. |
| `rqt` | `false` | Starts `rqt_image_view` on the overhead raw camera when true. |
| `yolo` | `false` | Starts `vision_perception/yolo_node` from this launch when true. |

The launch file does not set `ROS_DOMAIN_ID`. Use the same domain for Gazebo bridges, perception, map, robot, and ROS Bridge processes. [code]

## Published topics

QoS evidence: node publishers are created with depth 10, which gives `KEEP_LAST`, reliable, volatile under the current rclpy defaults [code]. D1 live CLI captures observed reliable/volatile for perception publishers, but ROS reported history depth as `UNKNOWN`. Camera bridge endpoints observed live were reliable/volatile; the initial overhead image query preceded publisher discovery. Do not treat unknown depth as a captured value.

Rates below distinguish SDF targets, recorder measurements, and the single D1 observation. Sim rates vary with RTF. H4 recorder results are retained as historical measurements; D1 `ros2 topic hz` captures did not produce stable rates for most topics.

| Topic | Type | Rate: wall / sim | QoS evidence | `frame_id` | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/surveillance_camera/image_raw` | `sensor_msgs/msg/Image` | SDF target 30 Hz sim; H4 measured 11.557 / 17.145 Hz. | D1's initial query preceded publisher discovery; this endpoint's QoS was not directly captured. Bridge reliability/volatility was observed on other endpoints; depth unknown. | `overhead_camera/camera_link/overhead_camera` [D1]. | Overhead native camera input. |
| `/corner_camera/image_raw` | `sensor_msgs/msg/Image` | SDF target 10 Hz sim; H4 measured 3.624 / 5.377 Hz. | Camera bridge reliable/volatile observed; depth unknown. | `corner_camera/camera_link/corner_camera` [D1]. | Corner native camera input. |
| `/surveillance_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | SDF sensor target 30 Hz sim; runtime wall/sim rate unverified. | Bridge reliable/volatile observed; depth unknown. | Unverified. | Bridged; `yolo_node` does not subscribe to CameraInfo. |
| `/corner_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | SDF sensor target 10 Hz sim; runtime wall/sim rate unverified. | Bridge reliable/volatile observed; depth unknown. | Unverified. | Bridged; `yolo_node` does not subscribe to CameraInfo. |
| `/clock` | `rosgraph_msgs/msg/Clock` | D1 one CLI sample: 539.3 messages/s wall; sim rate unverified. | Publisher reliable/volatile; `use_sim_time` subscriber best-effort/volatile observed; depth unknown. | No header. | Published only with `bridge_clock:=true`. |
| `/person_activity` | `std_msgs/msg/String` | Wall-clock target default 2.0 Hz; effective configured range 1.5-10 Hz. D1 all-message average 1.615 Hz; active-only cadence unverified. | Reliable/volatile observed; code depth 10. | No header. | D1: 199 valid strings; largest overall inter-message gap 21.952 s (not classified as active-only). |
| `/person_activity_json` | `std_msgs/msg/String` | One per inference; H4 measured 1.874 / 2.588 Hz. | Reliable/volatile observed; code depth 10. | No header. | JSON telemetry; Flutter app does not use it. |
| `/yolo/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | Bounded by `max_fps` (default 2.0 wall, hard cap 10); H4 measured 1.872 / 2.754 Hz. | Reliable/volatile observed; code depth 10. | `overhead_camera_link` [code, D1]. | Overhead native 640x480 JPEG, `format="jpeg"`, quality 75. |
| `/yolo/corner/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | Bounded by `max_fps` (default 2.0 wall, hard cap 10); H4 measured 1.874 / 2.590 Hz. | Reliable/volatile observed; code depth 10. | `corner_camera_link` [code, D1]. | Corner native 640x480 JPEG, `format="jpeg"`, quality 75. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Event-driven. D1 observed four idle-segment events across 88.969 sim s. | Reliable/volatile observed; code depth 10. | `map` [code, D1]. | Requires `/map`; see goal semantics and D1 coordinate results. |
| `/yolo/annotated/raw` | `sensor_msgs/msg/Image` | Not published by default; when enabled, bounded by `max_fps`. | Code depth 10, reliable/volatile. | `overhead_camera_link` [code]. | Enable with `publish_raw:=true`. |
| `/yolo/corner/annotated/raw` | `sensor_msgs/msg/Image` | Not published by default; when enabled, bounded by `max_fps`. | Code depth 10, reliable/volatile. | `corner_camera_link` [code]. | Enable with `publish_raw:=true`. |

**App-only camera transport:** `/surveillance_camera/image_raw/compressed` is not published by this project. The Flutter Camera Compress card is expected to create it from the overhead raw input [per Flutter contract]. D1 discovered `sensor_msgs/msg/CompressedImage` and a reliable/volatile publisher, but no message or rate was confirmed; subscribers/QoS remain unverified. See [measurements](docs/INTEGRATION_NOTES.md#d1-live-integration-verification).

## Consumed topics

| Topic | Type | Required/observed QoS | Behavior |
| :--- | :--- | :--- | :--- |
| `/surveillance_camera/image_raw` | `sensor_msgs/msg/Image` | `yolo_node` default subscription: depth 10, reliable/volatile [code]. The bridge endpoint itself was not directly captured in D1. | Overhead frame input; expected 640x480. |
| `/corner_camera/image_raw` | `sensor_msgs/msg/Image` | `yolo_node` default subscription: depth 10, reliable/volatile [code]. Bridge reliable/volatile was observed in D1. | Corner frame input; expected 640x480. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Required publisher: depth 1, reliable, transient-local [code/test helper]. | No map means no goal publication. Staying episode remains eligible and image callbacks retry; cooldown is not consumed. |
| `/clock` | `rosgraph_msgs/msg/Clock` | ROS `use_sim_time`; D1 saw best-effort/volatile subscription. | World launch publishes only with `bridge_clock:=true`. |

`yolo_node` does not subscribe to CameraInfo, TF, `/odom`, `/scan`, `/cmd_vel`, or the app's compressed camera transport. Calibration comes from `config/cameras.yaml`. [code]

## Message semantics

### Activity

- `/person_activity` contains exactly `Walking`, `Running`, or `Staying`; multiple valid tracks resolve by severity `Staying > Running > Walking`. [code]
- Nothing is published when there are no active tracks, all labels are `Unknown`, or there is no valid label. Repetition stops after 3 seconds without an inference update. [code]
- A valid label change is published immediately unless the 10 Hz minimum separation has not elapsed; otherwise it is pending until eligible. Repeats use `time.monotonic()`, default 2.0 Hz, clamped to 1.5-10.0 Hz; repeat interval is at most 0.6 s. Dwell and goal timing do not use this wall timer. [code]
- D1's all-message average was 1.615 Hz wall and the maximum gap across all emitted messages was 21.952 s. Retained telemetry cannot identify whether that gap occurred while a valid label was continuously classified. Do not use the aggregate as proof of the active-only interval. [verified, active-only interval unverified]

### Goals

- The same track must remain `Staying` for at least 5 seconds; image header stamps drive dwell and the goal header. One goal is emitted per Staying episode, with a global 15-second cooldown. [code]
- `/map` must arrive first with reliable, transient-local QoS. If absent, the episode stays armed, no cooldown is consumed, and later image callbacks retry while the track remains Staying. [code]
- The target is transformed from world metres to map metres. A cell is free only when its occupancy value is exactly `0`; otherwise the code searches for the nearest free cell center within 0.5 m. Unknown (`-1`) and occupied cells are not free. Indexing uses origin x/y, resolution, width/height, and row-major `iy * width + ix`; map-origin yaw is assumed zero. [code]
- The target is the median projected world position accumulated during that Staying episode, transformed to map coordinates before the free-space check. [code]
- Output is `frame_id="map"`, `z=0`, identity orientation (`x=y=z=0`, `w=1`). [code]
- D1 observed four valid idle-segment goals at image stamps 11.452, 34.882, 61.381, and 88.969 s. The first three were near `(0,0)`; the fourth was near `(0.8951,-0.0656)` after changing the YAML translation to `[1,0,0]`. A live parameter set to `[1,0,0]` was accepted but did not update the already-constructed GoalPoseManager. To change goal transforms, set `goal_pose.world_to_map` in YAML and restart the node. [verified]

### Annotated images and JSON

- Each camera is inferred from its own native frame using `predict(imgsz=640)`; no side-by-side composite or shared persistent tracker is used. [code]
- Detection boxes belong to that view. Fused global-track markers are projected back into both views, so overhead may show corner-only tracks. Labels include track ID, activity, and map `(x,y)` metres. [code]
- Both compressed outputs preserve native 640x480, use `format="jpeg"` and default quality 75. `max_fps` bounds inference and annotation; raw annotation topics are disabled unless `publish_raw:=true`. [code]
- `/person_activity_json` is a `String` containing `timestamp` and `entries`; entries include track ID, activity, speed, world/map coordinates, cameras, confidence, bbox, and classifier sample/window information. It is telemetry, not the Flutter activity contract. [code]

## World, coordinates, and cameras

### World reference

SDF world name is `surveillance_world`. Physics step is 0.001 s and configured real-time factor is 1.0; measured CPU RTF is lower. The SDF enables Physics, UserCommands, SceneBroadcaster, and Sensors (`ogre2`) systems. The 20x20 m floor is at z=0. Four 2.5 m-high perimeter walls have centerlines at x/y = +/-8 m and dimensions 16.2x0.2x2.5 m or 0.2x16.2x2.5 m. The room reference is **16x16 m**; the central partition is absent. [code]

| Object | Pose `(x,y,z,yaw)` | Geometry |
| :--- | :--- | :--- |
| NW pillar | `(-3.5, 3.5, 1.25, 0)` | Static 0.8x0.8x2.5 m box. |
| NE pillar | `(3.5, 3.5, 1.25, 0)` | Static 0.8x0.8x2.5 m box. |
| SW crate | `(-4.0, -4.5, 0.5, 0.2)` | Static 1.2x1.2x1.0 m box. |
| SE crate | `(4.5, -4.0, 0.4, -0.15)` | Static 1.5x1.0x0.8 m box. |

Lighting uses a directional sun at `(0,0,10)`, casts shadows, has diffuse 0.8/specular 0.2, and points `(-0.5,0.1,-0.9)`; scene ambient/background are 0.8. The animated actor uses Fuel `walk.dae`, `stand.dae`, and `run.dae` assets, each at scale 1.0. Actor route has no collision geometry. [code]

Actor cycle is 27 s: idle at `(0,0)` from 0-12 s; walk to `(0,3.5)` by 16 s at 0.875 m/s; run via `(2.5,3.5)` at 16.833333 s, then to `(2.5,-3.3)` at 19 s (3.000 and 3.138 m/s); walk back through `(2.5,0)` at 22 s, `(1.5,0)` at 23 s, `(1.5,1.25)` at 24.25 s, `(0,1.25)` at 25.75 s, and `(0,0)` at 27 s. [code]

Schematic only; coordinates in metres:

```text
                         north, y=+8
       +-----------------------------------------+
       | spawn (-6.2,6)   PNW      PNE  spawn(6.2,6)
       |                    actor path ->         |
       |                         |                |
       |                    (0,0)                |
       |                         |                |
       |   crate SW          run path    crate SE |
       +-----------------------------------------+
 west x=-8                                east x=+8

 PNW/PNE = pillars at (-3.5,3.5)/(3.5,3.5).
 The actor route is centered near x=0..2.5; the partition is absent.
 ```

Candidate robot base-center poses `(6.2,6.0,0)` and `(-6.2,6.0,0)` have computed minimum clearances of 1.7 m to a wall, about 3.115 m to the nearest pillar, 9.64/10.13 m to a crate, and 4.47/6.69 m to the actor route, respectively. These are geometric center-point distances, not a robot-footprint or spawn verification; the robot owner must check its model. [code geometry; spawn unverified, robot owner]

### Frames and world-to-map

World coordinates are metres. `world_to_map=[tx,ty,yaw_radians]` rotates the world point counter-clockwise, then translates:

```text
x_map = tx + cos(yaw)*x_world - sin(yaw)*y_world
y_map = ty + sin(yaw)*x_world + cos(yaw)*y_world
```

Worked example: if a yaw-zero map origin is the robot's starting world pose `(6.2,6.0)`, use `world_to_map=[-6.2,-6.0,0]`; world `(6.2,6.0)` maps to `(0,0)`. This is a calibration example, not a claim that a SLAM map's origin is at the spawn. The OccupancyGrid origin and `world_to_map` are distinct. D1 confirmed that changing the ROS parameter at runtime does not update the GoalPoseManager's copied transform; restart after changing YAML. [code, verified]

No world-to-map TF is published by this project. Any production map must match the final partition-free world and use origin yaw zero for current free-space indexing. [code]

### Cameras and projection

| Camera | Pose `(x,y,z,roll,pitch,yaw)` | Resolution | HFOV | Projection plane / bbox point | SDF rate |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Overhead | `(0,0,12.2,0,1.5708,1.5708)` | 640x480 | 1.50 rad | `z=1.5 m` / `center` | 30 Hz |
| Corner | `(-7,-7,2.3,0,0.25,0.7854)` | 640x480 | 1.05 rad | `z=0.0 m` / `bottom_center` | 10 Hz |

`cameras.yaml` supplies the projection intrinsics and pose; incoming image dimensions are checked. Each pixel ray intersects that camera's configured `plane_z`. The camera/SDF drift test compares pose, HFOV, and resolution. [code]

## Parameters

### `yolo_node` ROS parameters

Defaults and meanings in this table are [code].

| Parameter | Default | Unit | Meaning |
| :--- | :--- | :--- | :--- |
| `use_sim_time` | `true` (forced) | bool | Use ROS simulation time. |
| `overhead_topic` | `/surveillance_camera/image_raw` | topic | Overhead image input; default from YAML. |
| `corner_topic` | `/corner_camera/image_raw` | topic | Corner image input; default from YAML. |
| `output_compressed_topic` | `/yolo/annotated/compressed` | topic | Overhead JPEG output. |
| `corner_output_compressed_topic` | `/yolo/corner/annotated/compressed` | topic | Corner JPEG output. |
| `output_raw_topic` | `/yolo/annotated/raw` | topic | Optional overhead raw output. |
| `corner_output_raw_topic` | `/yolo/corner/annotated/raw` | topic | Optional corner raw output. |
| `activity_topic` | `/person_activity` | topic | Contract activity output. |
| `activity_json_topic` | `/person_activity_json` | topic | Inference telemetry. |
| `activity_publish_hz` | `2.0`, clamp 1.5-10.0 | wall Hz | Activity repeat target; non-finite value rejected; repeat interval <=0.6 s. |
| `goal_pose_topic` | `/goal_pose` | topic | Goal output. |
| `map_topic` | `/map` | topic | OccupancyGrid input. |
| `model` | `yolov8n.pt` | model ID/path | Ultralytics model argument. |
| `conf` | `0.15` | confidence | Person detection threshold. |
| `device` | `cpu` | device | Device passed to Ultralytics. |
| `window_sec` | `2.0` | s | Classifier speed-estimation window. |
| `staying_thresh` | `0.3` | m/s | Speeds below this classify as Staying. |
| `running_thresh` | `1.75` | m/s | Speeds at/above this classify as Running; between thresholds is Walking. |
| `smoothing_n` | `5` | samples | Majority-vote smoothing history. |
| `track_timeout_sec` | `2.0` | s | Classifier/global track inactivity timeout. |
| `fusion_dist_thresh` | `0.8` | m | Maximum overhead/corner detection separation for fusion. |
| `tracking_dist_gate` | `1.5` | m | Minimum association gate; age scale also applies. |
| `jpeg_quality` | `75` | JPEG quality | OpenCV compressed annotation quality. |
| `publish_raw` | `false` | bool | Enable both raw annotated topics. |
| `max_fps` | `2.0`, hard cap 10 | wall frames/s | Limits inference/annotation rate; must be positive finite. |
| `staying_dwell_sec` | `5.0` | s | Continuous Staying duration before goal; YAML default. |
| `goal_cooldown_sec` | `15.0` | s | Global minimum interval between goals; YAML default. |
| `world_to_map` | `[0.0,0.0,0.0]` | m, m, rad | World-to-map transform; YAML default. |

Classifier code constants, not ROS parameters: `min_samples=3`, `min_window_ratio=0.25`. Track association uses `max(tracking_dist_gate, 4.0 m/s * track_age)`. [code]

### `config/cameras.yaml` keys

Keys and values in this table are [code].

| Key | Default | Unit | Meaning |
| :--- | :--- | :--- | :--- |
| `overhead_camera.topic` | `/surveillance_camera/image_raw` | topic | Overhead image input. |
| `overhead_camera.pose` | `[0,0,12.2,0,1.5708,1.5708]` | m, rad | `[x,y,z,roll,pitch,yaw]`. |
| `overhead_camera.width` / `height` | `640` / `480` | px | Required native dimensions. |
| `overhead_camera.hfov` | `1.50` | rad | Horizontal field of view for projection intrinsics. |
| `overhead_camera.plane_z` | `1.5` | m | Ray-plane intersection height. |
| `overhead_camera.ground_point` | `center` | enum | Detection-box point for projection. |
| `corner_camera.topic` | `/corner_camera/image_raw` | topic | Corner image input. |
| `corner_camera.pose` | `[-7,-7,2.3,0,0.25,0.7854]` | m, rad | `[x,y,z,roll,pitch,yaw]`. |
| `corner_camera.width` / `height` | `640` / `480` | px | Required native dimensions. |
| `corner_camera.hfov` | `1.05` | rad | Horizontal field of view for projection intrinsics. |
| `corner_camera.plane_z` | `0.0` | m | Ray-plane intersection height. |
| `corner_camera.ground_point` | `bottom_center` | enum | Detection-box point for projection. |
| `goal_pose.staying_dwell_sec` | `5.0` | s | Default for the matching ROS parameter. |
| `goal_pose.goal_cooldown_sec` | `15.0` | s | Default for the matching ROS parameter. |
| `goal_pose.world_to_map` | `[0,0,0]` | m, m, rad | Default world-to-map transform. |

## Not provided

| Missing provider | Consumer must supply |
| :--- | :--- |
| Robot and robot bridges | Robot/navigation teammate supplies the robot model, spawn, and ROS/Gazebo bridges. |
| `/map` | Mapping/navigation supplies an OccupancyGrid matching this exact world, with reliable transient-local durability and yaw-zero origin. |
| TF | Robot/navigation supplies its robot and map transforms; this project publishes no TF. |
| Nav2, AMCL, mapping, `/odom`, `/scan`, `/cmd_vel` | Robot/navigation integration supplies these processes, topics, and frame relationships. |
| `/clock` with `bridge_clock:=false` | A combined robot/Gazebo integration must publish one shared simulation clock if consumers need ROS sim time. |

## Integration runbook and checklists

### 3.1 Choose deployment and shell

Use WSL Ubuntu 24.04, source the selected overlay, and set `ROS_DOMAIN_ID=30` in every launcher process. Choose Option A or B above; do not stack conflicting app/project overlays. [code, per Flutter contract]

### 3.2 Build and register

For the standalone checkout:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
colcon build --symlink-install
source install/setup.bash
pytest -q
```

The registration script is `scripts/register_with_ros.sh`; its VS Code task is `Surveillance_Sim: Register with ROS 2 (run once)`. The app's non-interactive shell must explicitly source `~/.surveillance_sim.bash` before resolving `ros2 launch`. [code]

### 3.3 Start project processes

World-only smoke run:

```bash
source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30 && ros2 launch surveillance_sim surveillance_world.launch.py headless:=true bridge_clock:=true
```

In another shell, start perception:

```bash
source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30 && ros2 run vision_perception yolo_node
```

For a map-dependent goal test, start the non-installed helper in a third shell:

```bash
source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30 && python3 src/vision_perception/test_tools/publish_test_map.py
```

The helper publishes a 100x100 all-free map at 0.1 m/cell with origin `(-5,-5)`, `frame_id=map`, depth 1, reliable/transient-local QoS. It is test-only and does not represent a production map. [code, verified]

### 3.4 Flutter dashboard test without a robot

1. Select the app's Gazebo card with package `surveillance_sim` and launch file `surveillance_world.launch.py`; use `headless:=true` for a no-GUI smoke test and `bridge_clock:=true` if the app/test nodes use ROS sim time. [code, per Flutter contract]
2. Start the `ROS Bridge`, `Camera Compress`, and `YOLO` cards. The app-contract start sequence is Gazebo, ROS Bridge, Camera Compress, then YOLO. [per Flutter contract]
3. Start the Flutter app's location/navigation mock for the UI portions. It is not a ROS `/map` or TF provider; it does not validate goal placement or navigation against this world. [per Flutter contract]
4. To check `/goal_pose`, separately start `publish_test_map.py` before YOLO. The helper's all-free map only tests the perception goal path; it does not match obstacles or prove a production map. [code]
5. Camera tab input is `/surveillance_camera/image_raw/compressed`. D1 saw the expected type and a publisher, but no output message or rate; do not sign off the Camera tab until a live message and `ros2 topic hz` are confirmed. [per Flutter contract, D1 verified type only]

App card command lines below use standalone Option B. With Option A, replace `source ~/.surveillance_sim.bash` with `source ~/wahab_ros_work/install/setup.bash`. Every card also needs `ROS_DOMAIN_ID=30`.

| Launcher card | One-line command | Evidence |
| :--- | :--- | :--- |
| Gazebo | `source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30 && ros2 launch surveillance_sim surveillance_world.launch.py headless:=true bridge_clock:=true` | [code, verified] |
| Camera Compress | `source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30 && ros2 run image_transport republish raw in:=/surveillance_camera/image_raw out/compressed:=/surveillance_camera/image_raw/compressed` | [per Flutter contract; D1 found a publisher/type, but exact argv was not retained and messages/rate are unverified] |
| ROS Bridge | `source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30 && ros2 launch rosbridge_server rosbridge_websocket_launch.xml` | [unverified, Flutter owner; app card/package command is external to this repository] |
| YOLO | `source ~/.surveillance_sim.bash && export ROS_DOMAIN_ID=30 && ros2 run vision_perception yolo_node` | [code, verified] |
| AMCL Localization | `unverified: robot-owned package, launch file, map path, and parameters` | [unverified, robot/navigation owner] |
| Custom Navigation | `unverified: robot/app-owned package, launch file, and parameters` | [unverified, robot/navigation and Flutter owners] |

The Camera Compress line is an unverified candidate invocation; D1 started `image_transport republish` and found the expected publisher/type, but its exact argv was not retained and no output message was confirmed. Validate the card's exact command before relying on it. The ROS Bridge, AMCL, and Custom Navigation command rows must be replaced or confirmed against the actual app/robot Launcher configuration; this repository cannot validate external card commands.

### 3.5 Combined robot/world launch

Launch one Gazebo world. Set this project's `bridge_clock:=false` when the robot bridge owns the shared `/clock`; do not start a second Gazebo instance. The robot launch must spawn into the running `surveillance_world`, not launch its own copy. [code, robot integration unverified]

Skeleton only; replace the marked robot package and launch arguments with the robot owner's actual interfaces:

```python
# Pseudocode only; not runnable until robot launch/package arguments are supplied.
world = IncludeLaunchDescription(
    surveillance_world_launch,
    launch_arguments={
        "headless": "false",
        "bridge_clock": "false",
        "yolo": "false",
    }.items(),
)
robot = IncludeLaunchDescription(
    robot_spawn_into_existing_gazebo_launch,
    launch_arguments={
        "world_name": "surveillance_world",
        "x": "6.2",
        "y": "6.0",
        "yaw": "0.0",
    }.items(),
)
return LaunchDescription([world, robot])
```

`surveillance_world_launch`, robot include name, and spawn arguments above are placeholders, not provided APIs. The candidate pose is a center-point geometry suggestion only; robot footprint, spawn support, and bridge topics are for the robot owner to verify. [unverified, robot/navigation owner]

### 3.6 Build and calibrate a matching map

Build/save the map from the final partition-free SDF, including the perimeter walls, pillars, crates, and final floor geometry. For mapping, use a known yaw-zero spawn and retain the actual world spawn coordinates. Calibrate `world_to_map` from known corresponding world/map points; do not assume the map origin equals the spawn. Ensure OccupancyGrid origin yaw is zero for current free-space indexing. The suggested spawn and worked transform are in [World reference](#world-reference) and [Frames and world-to-map](#frames-and-world-to-map). [code; mapping procedure unverified, robot/navigation owner]

The `/map` publisher must use `nav_msgs/msg/OccupancyGrid`, reliable/transient-local QoS, frame `map`, valid dimensions/resolution/origin, and free cells (`0`) around valid goal points. A volatile-only publisher may not satisfy the perception subscriber's late-join requirement. [code]

### 3.7 Flutter Launcher and topic selection

Select `surveillance_sim` plus `surveillance_world.launch.py` in the app Gazebo card. In a non-interactive launcher script, source `~/.surveillance_sim.bash` and export domain 30 before `ros2 launch`; an interactive `.bashrc` source is not a substitute for the app's generated shell. [per Flutter contract, code]

| App use | ROS topic | Type | QoS guidance |
| :--- | :--- | :--- | :--- |
| Activity | `/person_activity` | `std_msgs/msg/String` | Reliable/volatile observed for publisher; confirm the bridge/subscriber QoS in the app. |
| Overhead annotated | `/yolo/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | Reliable/volatile observed; confirm the app's ROS Bridge subscription profile. |
| Corner annotated | `/yolo/corner/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | Reliable/volatile observed; confirm the app's ROS Bridge subscription profile. |
| Goal | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Reliable/volatile observed; app may ignore if it does not expose goals. |
| Camera tab | `/surveillance_camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | D1 message rate and endpoint QoS unverified; inspect live endpoint before sign-off. |
| Diagnostic telemetry (not consumed by Flutter contract) | `/person_activity_json` | `std_msgs/msg/String` | Reliable/volatile observed. |

The Flutter endpoint is `ws://localhost:9090` [per Flutter contract]. Exact rosbridge-server card command, subscription QoS forwarding, and app-side behavior are external and remain unverified by this repository.

### 3.8 Integration acceptance checks

These are verification commands, not claims that a robot or app was available during D1.

| Check | Command | Expected result |
| :--- | :--- | :--- |
| Resolve project packages | `ros2 pkg prefix surveillance_sim && ros2 pkg prefix vision_perception` | Both package prefixes resolve in the selected overlay. [code] |
| Resolve launch arguments | `ros2 launch surveillance_sim surveillance_world.launch.py --show-args` | `world`, `headless`, `paused`, `bridge_clock`, `rqt`, and `yolo` appear. [verified] |
| Check app-facing message types | `ros2 topic type /person_activity && ros2 topic type /yolo/annotated/compressed && ros2 topic type /goal_pose` | `std_msgs/msg/String`, `sensor_msgs/msg/CompressedImage`, `geometry_msgs/msg/PoseStamped`. [code] |
| Confirm fresh activity rate | `ros2 topic hz /person_activity` | While a person has a valid classification, >=1.5 and <=10 wall-clock messages/s; no invalid strings. D1's active-only interval is unverified. [code contract, runtime pending] |
| Check camera transport | `ros2 topic echo --once /surveillance_camera/image_raw/compressed` then `ros2 topic hz /surveillance_camera/image_raw/compressed` | One JPEG `CompressedImage`, then a stable rate; D1 did not confirm either message or rate. [per Flutter contract, runtime pending] |
| Check map publisher | `ros2 topic info -v /map` | `nav_msgs/msg/OccupancyGrid`, reliable and transient-local publisher. [code] |
| Check map metadata | `ros2 topic echo --once /map` | `header.frame_id: map`, positive resolution/dimensions, correct world origin, yaw zero. [robot owner must provide] |
| Check calibration | `ros2 param get /yolo_node world_to_map` | Startup transform matches surveyed world/map points. For goal output, restart after YAML edits; D1 found runtime set does not update GoalPoseManager. [verified] |
| Check simulation clock | `ros2 topic echo --once /clock` | Clock advances when `bridge_clock:=true`, or when supplied by the combined robot Gazebo bridge. [code] |
| Check robot TF | `ros2 topic echo --once /tf` | Robot-owned map/odom/base transforms exist and match the yaw-zero mapping setup. Frame names are robot-specific. [unverified, robot owner] |
| Check build/tests | `colcon build --symlink-install && pytest -q` | Both packages build and tests pass. [verified, D1: 78 passed] |

### Perception contributor files and commands

| Path | Responsibility |
| :--- | :--- |
| `src/vision_perception/yolo/inference.py` | ROS node, inference orchestration, topic I/O, fusion and annotations. |
| `src/vision_perception/yolo/activity_publisher.py` | Wall-clock repeat state and stale inference cutoff. |
| `src/vision_perception/yolo/activity_classifier.py` | Speed window, thresholds, smoothing, classifier track timeout. |
| `src/vision_perception/yolo/tracking.py` | Age-scaled global-track association. |
| `src/vision_perception/yolo/ground_projection.py` | Configured camera-ray projection. |
| `src/vision_perception/yolo/goal_manager.py` | Dwell, cooldown, transform, map indexing and free-space nudge. |
| `src/vision_perception/yolo/annotated_stream.py` | Topic constants and native JPEG encoding. |
| `src/vision_perception/config/cameras.yaml` | Camera geometry/intrinsics and goal defaults. |
| `src/surveillance_sim/worlds/surveillance_world.sdf` | World geometry, sensors and actor route. |
| `src/vision_perception/test_tools/` | Non-installed test-map publisher and measurement runner. |

Run tests from the workspace root:

```bash
colcon build --symlink-install
pytest -q
python3 src/vision_perception/test_tools/measure_run.py --help
```

The test-map helper command is in section 3.3. `measure_run.py` is a non-installed measurement script; its `--help` output describes output-directory and map options. To add a camera, update the SDF sensor, launch bridge arguments, `cameras.yaml`, and the SDF-drift tests. To change the actor, update its SDF trajectories and run `test_actor_trajectory.py`; coordinate any geometry change with the map owner.

## Known limitations and open decisions

| Limitation / decision | Evidence and integration action |
| :--- | :--- |
| Annotated feeds are CPU/inference-limited. | H4 recorder measured about 1.87 wall Hz per stream; `max_fps` default is 2.0 wall Hz. D1 topic-hz capture was inconclusive. Keep UI tolerant of this cadence. [verified] |
| Overhead detector does not detect the actor in the recorded A2 frames. | Actor was about 15x16 px overhead; corner-derived fused markers are projected into overhead annotations. [verified] |
| Running recognition is weak. | H4 classified 4/25 run samples as Running and 6/25 as Unknown. Keep thresholds unchanged until an approved tuning study. [verified] |
| Track IDs fragment. | H4 saw five nearest-actor ID changes across three cycles. [verified] |
| Activity active-only rate needs an isolated confirmation. | Code repeats on wall time at default 2 Hz and clamps to 1.5-10 Hz. D1 measured 1.615 Hz overall and a 21.952 s largest gap across all emitted messages; active-only max gap is unverified. [code, verified, open] |
| Gazebo SIGINT teardown can leave a child or report a segfault. | Earlier teardown reported a segmentation fault after measurements; D1 saw a `gz sim` child at its final in-script check, absent in a later shell. Use process-level shutdown verification. [verified, cleanup detail open] |
| Map indexing assumes origin yaw zero; only value `0` is free. | Map producer must match this exact world and use transient-local QoS. [code] |
| RTF is below 1 on CPU. | D1 RTF was 0.63265; prior measured runs were also below 1. This reduces wall-clock sensor throughput. [verified] |
| No usable GPU in the recorded WSL setup. | PyTorch CUDA build 13.0 vs available driver capability 12.5; `torch.cuda.is_available()` was false. No GPU software was installed. A supported newer driver/runtime or a PyTorch wheel compatible with the supported driver and GPU architecture would need evaluation; do not change this workspace without approval. [verified] |
| Fuel network may be required on first launch. | Actor assets are remote DAE URLs in the SDF. [code] |
| Run animation shows foot glide. | Sampled run clips show glide at about 3.0-3.14 m/s; no animation/route tuning is included. [verified] |
| `/surveillance_camera/image_raw/compressed` output is not confirmed. | D1 found its type and publisher but no message/rate; Flutter owner must verify the Camera Compress card. [verified, open] |
| ROS Bridge and robot Launcher commands are external. | Confirm actual card command and QoS in the app/robot workspaces; this repository cannot verify them. [unverified, Flutter and robot owners] |

Detailed experiments, pre-C1b activity-rate history, D1 goal coordinates, and remaining measurements are in [docs/INTEGRATION_NOTES.md](docs/INTEGRATION_NOTES.md). Historical rates are intentionally not repeated here.

## Change control

Any change to topic names/types, QoS, message semantics, coordinate conventions, launch arguments, defaults, or ownership requires updating this file in the same change. Keep measurements and history in [docs/INTEGRATION_NOTES.md](docs/INTEGRATION_NOTES.md) and implementation checkpoints in [changelog.md](changelog.md).
