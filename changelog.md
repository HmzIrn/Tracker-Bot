# Changelog

All notable changes and verification outcomes are documented in this file.

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
  - `gz topic -l` discovered the following active simulation topics:
    ```text
    /clock
    /gazebo/resource_paths
    /stats
    /world/surveillance_world/clock
    /world/surveillance_world/dynamic_pose/info
    /world/surveillance_world/pose/info
    /world/surveillance_world/scene/deletion
    /world/surveillance_world/scene/info
    /world/surveillance_world/state
    /world/surveillance_world/stats
    /world/surveillance_world/light_config
    /world/surveillance_world/material_color
    ```
  - `gz topic -e -t /stats -n 1` verified simulation stepping and real-time factor:
    ```text
    sim_time {
      sec: 6
      nsec: 505000000
    }
    real_time {
      sec: 8
      nsec: 197695042
    }
    iterations: 6505
    real_time_factor: 1.0009198453378656
    step_size {
      nsec: 1000000
    }
    ```
- Outcome: **PASSED**. World `surveillance_world` loaded cleanly with real-time physics stepping.

#### Test 2: Paused Mode Verification
- Command:
  ```bash
  ros2 launch surveillance_sim surveillance_world.launch.py headless:=true paused:=true
  ```
- Verification Commands & Results:
  - Echoing `/world/surveillance_world/stats` confirmed:
    ```text
    paused: true
    ```
- Outcome: **PASSED**. Omitting `-r` started the simulation in paused state as expected.

#### Test 3: GUI Mode Launch Verification
- Command:
  ```bash
  ros2 launch surveillance_sim surveillance_world.launch.py headless:=false
  ```
- Verification Commands & Results:
  - Inspected active processes:
    ```text
    ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -r .../surveillance_world.sdf --force-version 8
    gz sim -r .../surveillance_world.sdf
    gz sim server
    gz sim gui
    ```
- Outcome: **PASSED**. Both Gazebo Sim server and GUI components initialized and executed cleanly.

#### Verification Failure Note & Resolution
- During an initial automated Python subprocess test, `ros2 launch` failed to resolve `surveillance_sim` because Python was invoked directly without `install/setup.bash` sourced in the calling subshell, leaving `AMENT_PREFIX_PATH` set only to `/opt/ros/jazzy`. Sourcing the workspace overlay `install/setup.bash` prior to launching resolved this as expected.
