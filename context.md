# Project Context: ROS 2 Indoor Surveillance Simulation

## Environment

- Windows host with WSL2 Ubuntu 24.04; active workspace: `C:\Users\hmz_i\SEM-5\IOT\ros2_ws` (`/mnt/c/Users/hmz_i/SEM-5/IOT/ros2_ws` in WSL).
- ROS 2 Jazzy, Gazebo Harmonic (`gz-sim` 8), `colcon`.
- ROS commands, builds, and tests run in WSL. GUI use requires WSLg; headless simulation is supported.
- Set `ROS_DOMAIN_ID=30` for simulation and perception processes. The workspace registration script intentionally does not set it.
- [`INTERFACES.md`](INTERFACES.md) is the stable integration contract; [`docs/INTEGRATION_NOTES.md`](docs/INTEGRATION_NOTES.md) holds measurements and history.

## Package Layout and Ownership

```text
ros2_ws/
|-- scripts/register_with_ros.sh
|-- run_sim.ps1
|-- src/
|   |-- surveillance_sim/
|   |   |-- launch/surveillance_world.launch.py
|   |   |-- worlds/surveillance_world.sdf
|   |   `-- models/
|   `-- vision_perception/
|       |-- config/cameras.yaml
|       |-- yolo/
|       |   |-- inference.py
|       |   |-- activity_classifier.py
|       |   |-- annotated_stream.py
|       |   |-- goal_manager.py
|       |   |-- ground_projection.py
|       |   |-- tracking.py
|       |   `-- test_*.py
|       `-- test_tools/  (not installed as ROS nodes)
|           |-- measure_run.py
|           `-- publish_test_map.py
|-- docs/INTEGRATION_NOTES.md
`-- changelog.md
```

`surveillance_sim` provides the world, cameras, actor, and `ros_gz_bridge` launch. `vision_perception` installs the `vision_perception yolo_node` executable and owns the camera perception, track association, classification, annotations, and `/goal_pose` publisher. No robot, localization, mapping, or navigation stack is part of this workspace.

## World and Actor

The final SDF is a 16 m x 16 m room with perimeter walls, two pillars, two crates, two cameras, and one animated actor. The previously used central partition is absent. Any map used for goal validation must match this partition-free world.

The looping actor cycle is 27 s:

- 0-12 s: idle at world `(0,0)`.
- 12-16 s: walk at 0.875 m/s.
- 16-16.833 s and 16.833-19 s: run at 3.000 and 3.138 m/s.
- 19-27 s: walk back with segment speeds 1.100, 1.000, 1.000, 1.000, and 1.000 m/s.

The actor's stand/walk/run DAE animation resources use Gazebo Fuel URLs and may require network access or a local Gazebo cache. Sampled runtime review found visible foot glide during the current fast run; no SDF route or animation tuning has been made.

## Cameras and Perception

| Camera | Pose `(x,y,z,roll,pitch,yaw)` | Image / HFOV | Configured rate | ROS topic |
| :--- | :--- | :--- | ---: | :--- |
| Overhead | `(0,0,12.2,0,1.5708,1.5708)` | 640x480 / 1.50 rad | 30 Hz | `/surveillance_camera/image_raw` |
| Corner | `(-7,-7,2.3,0,0.25,0.7854)` | 640x480 / 1.05 rad | 10 Hz | `/corner_camera/image_raw` |

`cameras.yaml` is the camera calibration source: image dimensions, HFOV, pose, projection `plane_z`, and bbox ground-point selection. Incoming image dimensions are checked at runtime, and `test_camera_sdf_drift.py` compares the SDF against YAML. Camera images and camera-info use `ros_gz_bridge parameter_bridge`; the legacy image bridge is not used.

The overhead view has weak person detections at the actor's small image size. Corner detections and fused global tracks are projected into both annotated views. Each camera uses its own 640x480 frame; no composite stream is created. Compressed outputs use JPEG quality 75 and default `max_fps=2.0` (hard cap 10); raw annotated topics are disabled unless `publish_raw:=true`.

The `ActivityClassifier` estimates metric speed from image-header timestamps using a 2.0 s window, `staying_thresh=0.3 m/s`, `running_thresh=1.75 m/s`, and smoothing. The current H4 runtime measured confusion in all actor segments, especially Running; see the integration notes before changing thresholds.

## ROS Topics and Time Behavior

- `/person_activity`: `std_msgs/msg/String`, exact labels `"Walking"`, `"Running"`, `"Staying"` with priority Staying > Running > Walking. No label is published when no person is classified.
- Activity repeats use wall time, default `activity_publish_hz=2.0`, clamp 1.5-10 Hz, and stop after 3 s without inference. D1 measured 1.615 Hz across all emitted messages and a 21.952 s largest overall gap; active-only cadence is unverified.
- `/yolo/annotated/compressed`: overhead-only 640x480 JPEG.
- `/yolo/corner/annotated/compressed`: corner-only 640x480 JPEG.
- `/goal_pose`: `geometry_msgs/msg/PoseStamped`, map-frame ground point with `z=0`, `w=1`, emitted once after a continuous Staying dwell of at least 5 s; global cooldown is 15 s.
- The Flutter Camera tab expects the externally republished `/surveillance_camera/image_raw/compressed` topic; D1 found its type but did not confirm a message/rate.
- `/map`: input only. A compatible transient-local OccupancyGrid must be supplied by the consuming integration. Without it, goals are deferred and retried while Staying.

`yolo_node` forces `use_sim_time=True`; image header stamps drive classifier and dwell timing. Launch `bridge_clock:=true` when other nodes need ROS `/clock`. Free-space indexing uses map origin position, resolution, width, height, and row-major cells; map-origin yaw is assumed zero.

## Setup and Commands

Prerequisites include ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, the message dependencies in `package.xml`, and Python packages NumPy <2, OpenCV, PyYAML, PyTorch, and Ultralytics. The node prepends `~/yolo_env` to the import path and falls back to `~/yolo_venv`. `yolov8n.pt` is not committed; Ultralytics can download it on first use. D1 observed model resolution/loading through node initialization in 20.23 s, but did not isolate download time or a persistent cache location. Current WSL PyTorch CUDA 13.0 does not match the available driver capability 12.5, so inference runs on CPU. No GPU software was installed.

Register or refresh the workspace after cloning/moving it:

```bash
bash scripts/register_with_ros.sh
# Optional after building manually:
bash scripts/register_with_ros.sh --no-build
# Reversible removal:
bash scripts/register_with_ros.sh --remove
```

It creates `~/.surveillance_sim.bash`, adds a marked source block to `~/.bashrc`, and backs up the original to `~/.bashrc.surveillance_sim.bak`. The VS Code task label is exactly **`Surveillance_Sim: Register with ROS 2 (run once)`**. It supports native Linux and Windows/WSL task execution.

Manual build and run:

```bash
source ~/.surveillance_sim.bash
export ROS_DOMAIN_ID=30
colcon build --symlink-install
pytest -q
ros2 launch surveillance_sim surveillance_world.launch.py headless:=true bridge_clock:=true
ros2 run vision_perception yolo_node
```

Launch defaults are GUI on, unpaused, `bridge_clock:=false`, and `yolo:=false`. Use `yolo:=true` to start the node through the launch file. The app uses the bare `ros2 run vision_perception yolo_node` command and stops it with SIGINT, then SIGKILL after 6 s.

## Historical Runtime Check and C1b Update

The H4 headless run used the registered shell, test map, `bridge_clock:=true`, and the bare node command. It measured 81.016 simulated seconds in 120.190 wall seconds (RTF 0.674); image-header and `/clock` spans differed by 0.063 s. Three valid map-frame goals were published, one in each idle interval, at 11.683, 35.377, and 61.678 s.

- Current corner detection was present in 208/210 inference samples (99.0%). The nearest actor track ID changed five times; nine samples contained multiple IDs.
- The 100 H4 activity messages used only allowed labels. That rate measurement predates the C1b wall-clock publisher fix; see the integration notes for the historical measurement and the later candidate run.
- Measured input rates were 17.145 Hz sim overhead and 5.377 Hz sim corner, below configured sensor rates. Inference and annotated output were about 1.87 Hz wall / 2.59 Hz sim.
- The single C1b run measured 1.749 Hz wall / 2.142 Hz sim over 18.052 sim seconds, with one idle goal at 10.792 s. That run used the initial executor-timer implementation; its 0.893 s maximum gap prompted a worker-thread change. D1 later measured the worker at 1.615 Hz across all emitted messages, with a 21.952 s largest overall gap; active-only cadence remains unverified because the saved telemetry did not retain the needed classification timing.
- Run-segment classification is poor: only 4/25 nearest-track samples were Running and six were Unknown. Idle and walking are also confused. Keep defaults until a tuning decision is made.
- Goals require an external `/map`; its origin yaw must be zero for current free-space indexing. Map production and `world_to_map` calibration belong to the consuming integration.
- Overhead detection is weak for the actor's pixel size. CPU-only inference and external Gazebo Fuel assets are additional runtime constraints.

The detailed confusion counts, topic-rate tables, historical 15 Hz camera experiment, and prior no-map check are in [`docs/INTEGRATION_NOTES.md`](docs/INTEGRATION_NOTES.md). Runtime helpers in `vision_perception/test_tools` are measurement utilities, not installed nodes.
