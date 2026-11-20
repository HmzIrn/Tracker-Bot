# Project Context: Simulated Indoor Surveillance System

## Environment Details
- **Operating System**: Windows host running Windows Subsystem for Linux 2 (WSL2)
  - **WSL Distribution**: Ubuntu 24.04 LTS (`Ubuntu-24.04`)
  - **WSL Linux Kernel**: 6.18.54.1-microsoft-standard-WSL2 (x86_64)
- **Active Workspace**:
  - Windows Host Path: `C:\Users\hmz_i\SEM-5\IOT\ros2_ws`
  - WSL Mount Path: `/mnt/c/Users/hmz_i/SEM-5/IOT/ros2_ws`
- **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`)
- **Gazebo Distribution**: Gazebo Harmonic (`gz-sim` version 8.15.0)
- **Display Server**: WSLg (X11 / Wayland on `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`, `/tmp/.X11-unix/X0`)
- **Rendering Architecture**: **Hybrid CPU/GPU Rendering**
  - **3D Sensor Rendering & Visualization (GPU)**: Hardware-accelerated via Mesa Direct3D 12 Gallium driver (`GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`) executing on the dedicated **NVIDIA Quadro P1000 (4 GB VRAM)** with OpenGL 4.6 Core Profile. The OGRE 2 sensor engine offloads camera frame synthesis directly to the GPU.
  - **Physics Simulation & ROS 2 Middleware (CPU)**: Gazebo's physics engine (`DART`), sensor system plugin orchestration, `ros_gz` bridging nodes, and DDS messaging execute on host CPU cores.
- **Build Tool**: `colcon`

## Installed ROS 2 & Gazebo Integration Packages
- `ros_gz`: Umbrella meta-package for ROS 2 and Gazebo integration.
- `ros_gz_sim`: Executables (`gz_sim.launch.py`, entity management) for launching and controlling Gazebo Harmonic.
- `ros_gz_bridge`: Bidirectional bridge between ROS 2 topics/services and Gazebo Transport (`parameter_bridge`).
- `ros_gz_image`: Dedicated transport bridge for camera image streaming between Gazebo and ROS 2 (`image_bridge`).
- `ros_gz_interfaces`: ROS 2 message definitions for Gazebo communication.
- Underlying vendor packages: `gz_sim_vendor`, `gz_sensors_vendor`, `gz_rendering_vendor`, `gz_physics_vendor`, `gz_tools_vendor`, etc.
- GUI / Visualization: `rqt_image_view`.

## Project Structure & Architecture Decisions
```text
ros2_ws/
├── changelog.md                                 # Chronological log of all modifications and test results
├── context.md                                   # Environment details, architecture, and package context
├── camera_snapshot.jpg                          # Verification snapshot captured directly from camera stream
├── yolo_snapshot.jpg                            # Verification snapshot of compressed annotated stream
└── src/
    └── surveillance_sim/                        # Simulation and perception package
        ├── CMakeLists.txt                       # ament_cmake build script exporting share resources and programs
        ├── package.xml                          # Package manifest specifying ros_gz, rclpy, std_msgs, sensor_msgs
        ├── launch/
        │   └── surveillance_world.launch.py     # Launch file starting Gazebo + image bridge + rqt + optional yolo
        ├── worlds/
        │   └── surveillance_world.sdf           # SDF 1.9 world description with camera and animated human
        ├── models/                              # Directory for custom models
        │   └── human/                           # Minimal animated human actor model
        │       ├── model.config                 # Model metadata
        │       └── meshes/                      # Low-poly skeletal animation meshes
        │           ├── stand.dae                # Idle animation
        │           ├── walk.dae                 # Walking animation
        │           └── run.dae                  # Running animation
        └── yolo/                                # Perception & activity classification module
            ├── __init__.py                      # Python package marker
            ├── activity_classifier.py           # Pure ROS-independent sliding-window classifier
            ├── inference.py                     # ROS 2 ByteTrack tracking & streaming node
            └── test_activity_classifier.py      # Pytest test suite for classifier
```

### World Design (`surveillance_world.sdf`)
- **SDF Version**: 1.9.
- **Portability**: All models, geometries, and materials are self-contained without hardcoded local paths.
- **Physics**: Configured with a `1ms` step size (`max_step_size=0.001`), real-time factor `1.0`.
- **System Plugins**:
  - `gz::sim::systems::Physics`: Advances physical simulation.
  - `gz::sim::systems::UserCommands`: Entity creation and management.
  - `gz::sim::systems::SceneBroadcaster`: Publishes scene state for GUI and bridges.
  - `gz::sim::systems::Sensors`: Sensor processing system using the `ogre2` rendering engine.
- **Scene Rendering**:
  - `<scene>` configured with `<ambient>0.8 0.8 0.8 1.0</ambient>` and `<background>0.8 0.8 0.8 1.0</background>`.
- **Environment Geometry**:
  - **Floor**: 20m × 20m ground plane with visual and collision properties.
  - **Lighting**: Directional light `sun` (`pose: 0 0 10`, diffuse `0.8 0.8 0.8`) with shadows enabled.
  - **Perimeter Walls**: Enclosing a 16m × 16m indoor room with 2.5m height and 0.2m thickness (`wall_north`, `wall_south`, `wall_east`, `wall_west`).
  - **Static Obstacles**:
    - `pillar_nw`: Structural column at (-3.5, 3.5, 1.25), size 0.8m × 0.8m × 2.5m.
    - `pillar_ne`: Structural column at (3.5, 3.5, 1.25), size 0.8m × 0.8m × 2.5m.
    - `partition_wall`: Central corridor wall at (0, -2.0, 1.0), size 4.0m × 0.2m × 2.0m.
    - `crate_sw`: Storage crate at (-4.0, -4.5, 0.5), size 1.2m × 1.2m × 1.0m.
    - `crate_se`: Storage crate at (4.5, -4.0, 0.4), size 1.5m × 1.0m × 0.8m.
  - **Fixed Corner Camera (`corner_camera`)**:
    - **Mount Pose**: `-7.0 -7.0 2.3 0 0.25 0.7854` (South-West corner, 2.3m elevation, ~14.3° downward pitch, 45° yaw aiming diagonally across the room towards (0, 0)).
    - **Coverage**: Overlooks all obstacles (`crate_sw` in foreground, `partition_wall` in midground, `pillar_nw` and `pillar_ne` in background).
    - **Sensor Configuration**:
      - Resolution: 640 × 480 pixels.
      - Update Rate: 10 Hz (lowered to save GPU resources; auxiliary camera).
      - Horizontal FOV: 1.05 radians (~60.16°).
      - Pixel Format: `R8G8B8`.
      - Clip Planes: near 0.1m, far 50.0m.
      - Gazebo Topic: `/corner_camera/image_raw` (Type: `gz.msgs.Image`).
      - Gazebo Camera Info Topic: `/corner_camera/camera_info` (Type: `gz.msgs.CameraInfo`).
  - **Fixed Overhead Surveillance Camera (`overhead_camera`)**:
    - **Mount Pose**: `0.0 0.0 12.2 0 1.5708 1.5708` (Mounted at room center looking straight down; image-up = world +Y, image-right = world +X).
    - **Coverage**: Complete top-down view of the 16m × 16m room with >0.5m perimeter margin.
      - Mathematical height & FOV derivation: With image 640 × 480 (aspect ratio 4:3) and horizontal FOV $\theta_h = 1.50\text{ rad}$ ($\theta_h/2 = 0.75\text{ rad}$, $\tan(0.75) \approx 0.9316$), the vertical half-FOV is $\tan(\theta_v/2) = (480/640)\tan(\theta_h/2) = 0.75 \times 0.9316 = 0.6987$. At mounting height $H = 12.2\text{ m}$, vertical floor coverage is $2 \times 12.2 \times 0.6987 \approx 17.05\text{ m}$ (giving $(17.05 - 16.0)/2 \approx 0.52\text{ m}$ margin along the room's North-South axis). Horizontal floor coverage is $2 \times 12.2 \times 0.9316 \approx 22.73\text{ m}$ (>3.3m margin East-West).
    - **Sensor Configuration**:
      - Resolution: 640 × 480 pixels.
      - Update Rate: 30 Hz.
      - Horizontal FOV: 1.50 radians (~85.94°).
      - Pixel Format: `R8G8B8`.
      - Clip Planes: near 0.1m, far 50.0m.
      - Gazebo Topic: `/surveillance_camera/image_raw` (Type: `gz.msgs.Image`).
      - Gazebo Camera Info Topic: `/surveillance_camera/camera_info` (Type: `gz.msgs.CameraInfo`).
  - **Animated Human Subject (`human_subject`)**:
    - **Architecture**: Kinematic `<actor>` model with skeletal animation clips, avoiding rigid-body physics contact solver overhead.
    - **Asset Storage**: Portable meshes installed locally in `models/human/meshes/` (`stand.dae`, `walk.dae`, `run.dae`).
    - **States & Animations**:
      - `idle`: Natural standing pose (`stand.dae`) in the center of the room within direct camera view.
      - `walk`: Steady walking locomotion (`walk.dae`) at ~1.0 m/s along the midground corridor.
      - `run`: Dynamic running gait (`run.dae`) at ~2.5 m/s across the surveillance area.
    - **Trajectory Cycle**: Defined via `<script>` with `<loop>true</loop>` (total 27.0s):
      - 0.0s - 12.0s: **Idle (Staying)** at `(0, 0, 1.0)`, yaw 45° facing corner camera.
      - 12.0s - 20.0s: **Walk** from `(0, 0, 1.0)` to `(2.5, 2.5, 1.0)` then `(0.5, 4.5, 1.0)`.
      - 20.0s - 27.0s: **Run** from `(0.5, 4.5, 1.0)` to `(-2.5, 1.5, 1.0)` and loop back to origin `(0, 0, 1.0)`.
    - **Performance Characteristics**: Hardware-accelerated vertex skinning on the NVIDIA GPU via D3D12 with negligible CPU load (~0% physics collision solver impact).

### ROS 2 Integration & Launch Configuration (`surveillance_world.launch.py`)
- Resolves the world file path dynamically via `FindPackageShare('surveillance_sim')`.
- Includes `gz_sim.launch.py` from `ros_gz_sim`.
- Spawns bridging nodes via `ros_gz_bridge/parameter_bridge`:
  - Bridges Gazebo `/surveillance_camera/image_raw` (`gz.msgs.Image`) to ROS 2 `/surveillance_camera/image_raw` (`sensor_msgs/msg/Image`).
  - Bridges `/surveillance_camera/camera_info` (`gz.msgs.CameraInfo`) to ROS 2 (`sensor_msgs/msg/CameraInfo`).
  - Bridges Gazebo `/corner_camera/image_raw` (`gz.msgs.Image`) to ROS 2 `/corner_camera/image_raw` (`sensor_msgs/msg/Image`).
  - Bridges `/corner_camera/camera_info` (`gz.msgs.CameraInfo`) to ROS 2 (`sensor_msgs/msg/CameraInfo`).
  - Optionally bridges `/clock` when `bridge_clock:=true`.
  - NOTE: `ros_gz_image/image_bridge` removed to avoid generating redundant duplicate compressed topics.
  - `rqt_image_view`: Optional GUI node conditionally launched when `rqt:=true` viewing `/surveillance_camera/image_raw`.
- Supported Launch Arguments:
  - `world`: Path to the SDF world file. Defaults to installed `share/surveillance_sim/worlds/surveillance_world.sdf`.
  - `headless`: `true` or `false` (default: `false`). When `true`, passes `-s` to Gazebo.
  - `paused`: `true` or `false` (default: `false`). When `false`, passes `-r` to Gazebo.
  - `bridge_clock`: `true` or `false` (default: `false`). Bridges Gazebo `/clock` to ROS 2.
  - `rqt`: `true` or `false` (default: `false`). Launches `rqt_image_view` viewing `/surveillance_camera/image_raw`.
  - `yolo`: `true` or `false` (default: `false`). Starts the YOLO tracking and activity classification node.

### YOLO Perception & Activity Classification Architecture
The perception pipeline consumes both `/surveillance_camera/image_raw` and `/corner_camera/image_raw`:
- **Detection and Tracking**: Ultralytics YOLO detects persons in each camera. Projected to the floor using each camera's calibrated pose, fused across cameras, then associated across frames to maintain global IDs.
- **Pure Activity Classifier (`ActivityClassifier`)**:
  - Independent unit-testable class tracking projected world-ground coordinates for each global track ID.
  - Uses simulation timestamps from image headers.
  - Calculates speed in meters per second over a sliding time window (`window_sec`).
  - Classifies states into:
    - `Staying`: $v < \text{staying\_thresh}$
    - `Walking`: $\text{staying\_thresh} \le v < \text{running\_thresh}$
    - `Running`: $v \ge \text{running\_thresh}$
    - `Unknown`: Insufficient track history (< `min_samples=3`).
    - `None`: No human detected in field of view.
  - Majority-vote smoothing over the last $N$ predictions (`smoothing_n`) eliminates frame-to-frame gait cycle oscillation and jitter.
  - Stale tracks are pruned automatically after `track_timeout_sec`.
- **CV Bridge Avoidance**: Image conversions are implemented directly using raw `numpy` buffer slicing and OpenCV (`cv2`) without `cv_bridge` to prevent numpy 1.x vs 2.x ABI crashes.
- **Annotated Streaming**: Side-by-side frame (overhead + corner views, at most 640x480) published as JPEG `sensor_msgs/msg/CompressedImage` on `/yolo/annotated/compressed` (capped at 10 fps).
- **Goal Pose**: `/goal_pose` (`geometry_msgs/msg/PoseStamped`) published on staying event >= 5s.

### ROS 2 Topics
| Topic | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/surveillance_camera/image_raw` | `sensor_msgs/msg/Image` | Bridged | Raw RGB8 640x480 30 FPS overhead camera feed from Gazebo bridge |
| `/surveillance_camera/camera_info`| `sensor_msgs/msg/CameraInfo` | Bridged | Overhead camera calibration info |
| `/corner_camera/image_raw` | `sensor_msgs/msg/Image` | Bridged | Auxiliary corner camera feed (10 FPS) from Gazebo bridge |
| `/corner_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Bridged | Corner camera calibration info |
| `/yolo/annotated/compressed`| `sensor_msgs/msg/CompressedImage` | Published | JPEG-compressed annotated frame for web/Flutter streaming (max 640x480, max 10 fps) |
| `/person_activity` | `std_msgs/msg/String` | Published | Case-sensitive contract string ("Walking", "Running", "Staying") |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Published | Person foot position in map coordinates upon staying >= 5s |
| `/clock` | `rosgraph_msgs/msg/Clock` | Bridged | Simulation clock (when `bridge_clock:=true`) |

### Node Parameters (`inference.py`)
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `model` | string | `yolov8n.pt` | YOLO model weights name or path |
| `conf` | double | `0.15` | Detection confidence threshold (tuned for Gazebo Collada meshes) |
| `window_sec` | double | `1.0` | Sliding window duration for speed estimation (seconds) |
| `staying_thresh` | double | `0.3` | Threshold between Staying and Walking (m/s) |
| `running_thresh` | double | `1.75` | Threshold between Walking and Running (m/s) |
| `device` | string | `cpu` | Ultralytics inference device |
| `fusion_dist_thresh` | double | `0.8` | Maximum distance to fuse detections across cameras (meters) |
| `tracking_dist_gate` | double | `1.5` | Maximum distance to associate a detection to an existing track (meters) |
| `smoothing_n` | int | `5` | Majority-vote smoothing window size (history length) |
| `track_timeout_sec` | double | `2.0` | Inactivity duration before dropping lost tracks (seconds) |
| `jpeg_quality` | int | `80` | JPEG compression quality (1–100) |
| `publish_raw` | bool | `true` | Toggle publishing raw `/camera/image_annotated` |
| `max_fps` | double | `2.0` | Maximum inference rate throttle in wall-clock FPS |

### How to Run & Verify

1. **Launch Simulation with Perception Pipeline**:
   ```bash
   ros2 launch surveillance_sim surveillance_world.launch.py yolo:=true
   ```
   Or run the VS Code task **ROS 2: Launch Surveillance Simulation with YOLO**. Use **ROS 2: View YOLO Feed (RQt)** to display `/camera/image_annotated`; the compressed topic carries `CompressedImage` messages and is intended for compatible clients, not the standard RQt Image viewer.
2. **Launch Simulation Headless & Run Perception Node Manually**:
   ```bash
   # Terminal 1: Simulation
   ros2 launch surveillance_sim surveillance_world.launch.py headless:=true

   # Terminal 2: YOLO Node
   ros2 run surveillance_sim inference.py
   ```
3. **Run Unit Tests (Pure Activity Classifier)**:
   ```bash
   pytest src/surveillance_sim/yolo/test_activity_classifier.py -v
   ```
4. **Inspect Topics & Streams**:
   ```bash
   # Echo plain badge topic
   ros2 topic echo /person_activity

   # Echo JSON telemetry
   ros2 topic echo /person_activity_json

   # Check compressed frame bandwidth & rate
   ros2 topic hz /yolo/annotated/compressed
   ros2 topic bw /yolo/annotated/compressed
   ```

## VS Code Task Automation (`.vscode/tasks.json`)
The workspace includes dual-platform automated tasks to execute common workflows directly from VS Code (`Terminal -> Run Task...` or `Ctrl+Shift+P -> Tasks: Run Task`). Each task defines a native Linux execution block (when connected via VS Code WSL Remote) and a `"windows"` configuration block using `wsl -d Ubuntu-24.04 --cd "${workspaceFolder}"` for native Windows execution:
- **`ROS 2: Launch Surveillance Simulation (GUI)`**: Builds if not yet built, sources the workspace, and runs `ros2 launch surveillance_sim surveillance_world.launch.py`, opening the Gazebo GUI window with the full simulation and ROS 2 bridges active via WSLg.
- **`ROS 2: Launch Surveillance Simulation with Camera Preview (GUI + RQt)`**: Launches the simulation with both Gazebo GUI and `rqt_image_view` displaying the camera feed.
- **`ROS 2: Launch Surveillance Simulation with YOLO`**: Starts Gazebo, both camera bridges, and the inference node in the YOLO virtual environment.
- **`ROS 2: Run YOLO Inference Node`**: Runs `inference.py` using the WSL virtual environment `/home/hmz/yolo_venv`.
- **`ROS 2: View YOLO Feed (RQt)`**: Launches `rqt_image_view` subscribed to `/camera/image_annotated`.
- **`Gazebo: Open Surveillance World (Standalone GUI)`**: Directly opens `surveillance_world.sdf` in Gazebo Sim GUI without ROS 2 nodes.
- **`ROS 2: Build workspace`**: Runs `colcon build --symlink-install` (configured as default build task).
- **`ROS 2: List active topics`**: Inspects all active ROS 2 topics.
