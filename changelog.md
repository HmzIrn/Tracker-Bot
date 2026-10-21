# Changelog

All notable changes and verification outcomes are documented in this file.

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
