# Integration Notes: Flutter Dashboard & Surveillance System Contract

## 1. Overview & Architecture Split
The workspace has been factored into two distinct ROS 2 packages adhering to the project's responsibilities:
- **`surveillance_sim`**: Owns the simulation environment, physical SDF description, lighting, obstacles, actor kinematic animation clips, and Gazebo-to-ROS bridges.
- **`vision_perception`**: Owns multi-camera YOLO perception, ground projection, metric activity classification, goal pose emission, and annotated frame streaming.

## 2. Topic & Contract Specification

| Contract Topic | Message Type | Frequency / Behavior | Description |
| :--- | :--- | :--- | :--- |
| `/person_activity` | `std_msgs/msg/String` | 1–10 Hz (when visible) | Strictly `"Walking"`, `"Running"`, or `"Staying"` (case-sensitive). Publishes **nothing** when no person is visible. With multiple persons, publishes the most serious label (`Staying > Running > Walking`). |
| `/yolo/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | <= 10 fps | Overhead-only native 640x480 JPEG (quality 75). Includes native detection boxes and magenta projected global-track markers labeled with track ID, activity, and map `(x, y)` metres. |
| `/yolo/corner/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | <= 10 fps | Corner-only native 640x480 JPEG (quality 75), with the same detection-box and projected-track annotations. |
| `/yolo/annotated/raw` | `sensor_msgs/msg/Image` | Optional, disabled by default | Native overhead annotated frame; enabled with `publish_raw:=true`. |
| `/yolo/corner/annotated/raw` | `sensor_msgs/msg/Image` | Optional, disabled by default | Native corner annotated frame; enabled with `publish_raw:=true`. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Event-driven (once per episode) | Emitted once when a tracked person dwells continuously in `"Staying"` for >= 5.0 s. Frame is `"map"`, `z=0`, `w=1.0`. Enforces a 15.0 s global cooldown. Requires `/map`; defers and retries while Staying if no map is available. |
| `/surveillance_camera/image_raw` | `sensor_msgs/msg/Image` | ~30 Hz | Raw overhead camera stream from room center `(0.0, 0.0, 12.2)` looking straight down. |
| `/corner_camera/image_raw` | `sensor_msgs/msg/Image` | ~10 Hz | Auxiliary corner camera stream from SW mount `(-7.0, -7.0, 2.3)`. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Latched (`TRANSIENT_LOCAL`) | Subscribed by `yolo_node` to verify goal pose free space (`cell == 0`). A goal is deferred while no map is available. |

The occupancy grid conversion uses its origin position, resolution, width, and height (row-major indexing); origin yaw is assumed to be zero. If no map has arrived at a Staying dwell/cooldown event, the episode remains armed and the pose is retried on later image frames, with throttled warnings. Dwell and cooldown use image header timestamps rather than the ROS node clock.

### Camera Projection Calibration
- `vision_perception/config/cameras.yaml` is the source for each camera's pose, width, height, HFOV, projection plane, and ground-point selection. The overhead camera uses `plane_z: 1.5` with a bbox center; the corner camera uses `plane_z: 0.0` with the bbox bottom-center.
- `yolo_node` checks incoming image dimensions against that calibration. `test_camera_sdf_drift.py` also compares both cameras' SDF pose, HFOV, and image resolution to YAML.
- Integer-pixel round-trip error using the known room-center waypoint `(0, 0)` and near-wall waypoint `(0.5, 4.5)` is approximately **0.00004 m** overhead at center and **0.0682 m** from the corner camera near the wall.

### Activity Classifier Sampling
- The configured `window_sec` is 2.0 s (`staying_thresh` and `running_thresh` unchanged). Telemetry includes each track's `classifier_samples` and active window duration.
- On the 42.07 s runtime sample at `max_fps=2.0`, 11/33 track updates had fewer than three samples within 1.0 s (median 3, max 4); the corresponding 2.0 s replay had 9/33 below three (median 5, max 6). Replay confusion did not materially improve, so additional speed-threshold changes are not justified by this sample.

### Environment & Process Contract
- **ROS_DOMAIN_ID**: All nodes must run with `export ROS_DOMAIN_ID=30`.
- **Simulation Time**: `yolo_node` automatically overrides `use_sim_time=True` in its node constructor and utilizes image header stamps for all time math.
- **Node Invocation**: The node can be launched with bare command `ros2 run vision_perception yolo_node` with no CLI arguments and stdin piped.
- **Process Cleanup Patterns**:
  - YOLO: `yolo_node`
  - Gazebo & bridges: `gz sim|parameter_bridge|robot_state_publisher|turtlebot3_gazebo`

## 3. Deprecation of `image_bridge`
The previous transport bridge `ros_gz_image image_bridge` has been **completely removed**. It caused redundant, unconfigurable compressed image topic publications that conflicted with network bandwidth and Flutter expectations. Both cameras (`/surveillance_camera` and `/corner_camera`) now use `ros_gz_bridge parameter_bridge` for both `image_raw` and `camera_info`.

## 4. `world_to_map` Frame Calibration
The Cartographer/Nav2 map origin may not align with Gazebo's world origin `(0, 0, 0)`. The `yolo_node` applies a 2D SE(2) rigid-body transform from Gazebo world coordinates to Cartographer map coordinates:
$$\begin{bmatrix} x_{map} \\ y_{map} \end{bmatrix} = \begin{bmatrix} \cos(\psi) & -\sin(\psi) \\ \sin(\psi) & \cos(\psi) \end{bmatrix} \begin{bmatrix} x_{world} \\ y_{world} \end{bmatrix} + \begin{bmatrix} t_x \\ t_y \end{bmatrix}$$
where `world_to_map: [tx, ty, yaw_radians]`.

### How to Determine `world_to_map`:
1. Start the simulation and your teammate's Cartographer / SLAM node to build or load the map.
2. In RViz2, set the Fixed Frame to `map`.
3. Locate the center of the 16x16m room (where the overhead camera is mounted directly above at `(0, 0)`):
   - Read the map `(x, y)` coordinate of the center point using the RViz "Publish Point" or cursor tool. This gives `tx` and `ty`.
   - Measure the angular deviation between world +X (East wall) and map +X. This gives `yaw_radians`.
4. Update `world_to_map` without touching any code:
   - In `config/cameras.yaml` under `goal_pose: world_to_map: [tx, ty, yaw]`
   - Or pass via CLI / launch parameter: `ros2 run vision_perception yolo_node --ros-args -p world_to_map:="[<tx>, <ty>, <yaw>]"`

## 5. Milestone 11 Addendum Report (A1-A6)

### A1 — Independent annotated streams
- Outputs are native-resolution 640x480 overhead and corner JPEGs at quality 75; side-by-side composition is removed. Each is capped at 10 fps (the selected runtime default is 2 fps). Raw annotated topics remain disabled by default.
- YOLO receives each original camera frame independently at `imgsz=640`; no persistent tracker is shared. Orange boxes show native detections; magenta markers show fused global tracks projected into either view, including corner-only detections overlaid on overhead.
- Unit tests decode both streams at 640x480 and assert the two topic names. No separate live annotated-topic rate measurement was retained.

### A2 — Overhead detection experiment
- Frames at simulation stamp 3.0 s: [overhead](../a2_overhead_frame.jpg), [corner](../a2_corner_frame.jpg). Actor pixel extent was approximately 15x16 overhead and 14x80 corner (corner actor partially occluded).
- CPU, confidence 0.15; values below are person detections per saved frame:

  | Model | `imgsz=640` overhead/corner | `imgsz=960` overhead/corner | `imgsz=1280` overhead/corner |
  | :--- | :--- | :--- | :--- |
  | YOLOv8n | 0 / 0 | 0 / 1 (`0.211`, 13.1x55 px) | 0 / 1 (`0.419`, 14.0x56 px) |
  | YOLOv8s | 0 / 0 | 0 / 1 (`0.228`, 12.9x56 px) | 0 / 2 (`0.471`, 13.5x56 px; `0.334`, 13x81 px) |
  | YOLOv8m | 0 / 0 | 0 / 0 | 0 / 1 (`0.399`, 14.1x55 px) |
- Decision applied: overhead detection remains unchanged; A1 overlays corner/global tracks onto overhead. Do not infer sensor/model defaults from this single, partly occluded sample.

### A3 — Projection
- YAML intrinsics match the SDF: overhead 640x480 / HFOV 1.50 rad / `plane_z=1.5`; corner 640x480 / HFOV 1.05 rad / `plane_z=0.0`. Runtime image dimensions are checked against calibration.
- Integer-pixel geometric round-trip errors against the room-center `(0,0)` and near-wall `(0.5,4.5)` waypoints were 0.00004 m overhead and 0.06817 m corner. These are geometry/quantization errors, not independent YOLO localization errors; the corner actor was occluded in the saved frame and overhead YOLO did not detect it.

### A4 — Sampling and activity classification
- At `max_fps=2.0`, 42.07 sim seconds yielded 33 track updates across six global IDs. At a 1.0 s window sample count min/median/max was 1/3/4; 11/33 updates had fewer than three samples. At 2.0 s, the same recorded observations gave 1/5/6 and 9/33 below three. `window_sec` is now 2.0; speed thresholds were not changed.
- Confusion counts (true rows; predicted columns Staying / Walking / Running / Unknown):

  | Window | Staying | Walking | Running |
  | :--- | :--- | :--- | :--- |
  | 1.0 s | 0 / 0 / 0 / 2 | 0 / 8 / 0 / 5 | 0 / 15 / 1 / 2 |
  | 2.0 s replay | 0 / 0 / 0 / 2 | 0 / 8 / 0 / 5 | 0 / 16 / 0 / 2 |

- The replay did not improve the confusion matrix; worsening is concentrated in Running being labeled Walking. With the 2 fps throttle, observed telemetry rate was about 2.47 Hz simulated and RTF about 0.5. CPU throughput did not justify raising the throttle toward 5 fps.
- PyTorch is built for CUDA 13.0; the WSL driver reports 12.5 and CUDA is unavailable. Resolution: use a driver supported by the Quadro P1000 and CUDA 13, if available, or a PyTorch wheel compatible with its supported driver/runtime and Pascal architecture. No GPU software was installed.

### A5 — Goal pose and map
- Without `/map`, the node warns (throttled), publishes no goal, and retries on later image frames while Staying; cooldown and episode state are preserved. Dwell/cooldown use image header stamps.
- Tests cover frozen node time, retry without firing, and nonzero map origin with 0.2 m resolution. Occupancy indexing is row-major with zero origin yaw assumed.
- The headless run did not include a map publisher, so live goal publication/free-space behavior remains unverified; code-level tests pass.

### A6 — Summary and open decisions
- Each A1-A5 checkpoint passed `colcon build --symlink-install` and pytest (final: **29 passed**). The one successful headless capture ran **42.87 sim seconds** before the 110 s wall guard, with RTF approximately **0.5**; it did not reach the requested ~60 sim seconds.
- Keep current sensor resolution, model default, confidence, and speed thresholds pending a better-visibility run. Decide whether to pursue a P1000-compatible CUDA/PyTorch pairing; ensure the out-of-scope integration provides `/map`.
- If empirical plane calibration beyond the reported geometric round-trip is required, provide clear actor frames at the center and near-wall waypoints; the saved corner view is occluded and overhead detections were zero.

## 5. Teammate Handoff & Workspace Integration

### Packages to Copy into Teammate's Workspace
Copy the following two directories into `~/wahab_ros_work/src/`:
```bash
cp -r src/surveillance_sim ~/wahab_ros_work/src/
cp -r src/vision_perception ~/wahab_ros_work/src/
```
Build the teammate workspace:
```bash
colcon build --symlink-install --packages-select surveillance_sim vision_perception
```

### Thin Wrapper Launch File Snippet
Below is a ready-to-use launch file snippet that includes our surveillance world from any external robot launch (e.g. `turtlebot3_sim_world.launch.py`).
Save this inside your own meta-launch package (do **not** modify `turtlebot3_gazebo` directly):

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    surveillance_sim_share = FindPackageShare('surveillance_sim').perform()
    world_launch_path = os.path.join(
        surveillance_sim_share, 'launch', 'surveillance_world.launch.py'
    )

    # 1. Include Surveillance Gazebo World & Camera Bridges
    # Note: bridge_clock is set to false because TurtleBot3's launch bridges /clock
    surveillance_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_path),
        launch_arguments={
            'headless': LaunchConfiguration('headless', default='false'),
            'bridge_clock': 'false',
        }.items()
    )

    # 2. YOLO Node (started alongside or conditionally)
    yolo_node = Node(
        package='vision_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('yolo', default='true')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless'),
        DeclareLaunchArgument('yolo', default_value='true', description='Start YOLO perception node'),
        surveillance_world,
        yolo_node,
    ])
```
