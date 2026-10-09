# Project Context: Simulated Indoor Surveillance System

## Environment Details
- **Operating System**: Linux (Ubuntu 24.04 LTS / Linux 6.8.0-52-generic x86_64)
- **Active Workspace**: `/home/hsa/Public/Sem-5/IOT/Project/ros2_ws`
- **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`)
- **Gazebo Distribution**: Gazebo Harmonic (`gz-sim` version 8.15.0)
- **Display Server**: X11 (`DISPLAY=:0`)
- **Build Tool**: `colcon`

## Installed ROS 2 & Gazebo Integration Packages
- `ros_gz`: Umbrella meta-package for ROS 2 and Gazebo integration.
- `ros_gz_sim`: Executables (`gz_sim.launch.py`, entity management) for launching and controlling Gazebo Harmonic.
- `ros_gz_bridge`: Bidirectional bridge between ROS 2 topics/services and Gazebo Transport.
- `ros_gz_image`: Transport bridge for image streaming between Gazebo and ROS 2.
- `ros_gz_interfaces`: ROS 2 message definitions for Gazebo communication.
- Underlying vendor packages: `gz_sim_vendor`, `gz_sensors_vendor`, `gz_rendering_vendor`, `gz_physics_vendor`, `gz_tools_vendor`, etc.

## Project Structure & Architecture Decisions
```text
ros2_ws/
├── changelog.md                                 # Chronological log of all modifications and test results
├── context.md                                   # Environment details, architecture, and package context
└── src/
    └── surveillance_sim/                        # Simulation package
        ├── CMakeLists.txt                       # ament_cmake build script exporting share resources
        ├── package.xml                          # Package manifest specifying ros_gz_sim dependency
        ├── launch/
        │   └── surveillance_world.launch.py     # Launch file starting Gazebo with arguments
        ├── worlds/
        │   └── surveillance_world.sdf           # SDF 1.9 world description
        └── models/                              # Directory for custom models (future expansion)
```

### World Design (`surveillance_world.sdf`)
- **SDF Version**: 1.9.
- **Portability**: All models, geometries, and materials are self-contained without hardcoded local paths.
- **Physics**: Configured with a `1ms` step size (`max_step_size=0.001`), real-time factor `1.0`.
- **System Plugins**:
  - `gz::sim::systems::Physics`: Advances physical simulation.
  - `gz::sim::systems::UserCommands`: Entity creation and management.
  - `gz::sim::systems::SceneBroadcaster`: Publishes scene state for GUI and bridges.
  - `gz::sim::systems::Sensors`: Sensor processing system (Ogre2 engine configured for upcoming sensor integrations).
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

### ROS 2 Launch Configuration (`surveillance_world.launch.py`)
- Resolves the world file path dynamically via `FindPackageShare('surveillance_sim')`.
- Includes `gz_sim.launch.py` from `ros_gz_sim`.
- Supported Launch Arguments:
  - `world`: Path to the SDF world file. Defaults to installed `share/surveillance_sim/worlds/surveillance_world.sdf`.
  - `headless`: `true` or `false` (default: `false`). When `true`, passes `-s` to Gazebo, launching server only without GUI.
  - `paused`: `true` or `false` (default: `false`). When `false`, passes `-r` to Gazebo to immediately run the simulation on start. If `true`, omission of `-r` starts the simulation paused.
