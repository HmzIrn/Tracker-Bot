# Changelog

All notable changes and verification outcomes are documented in this file.

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
