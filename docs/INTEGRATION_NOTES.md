# Integration Notes: Flutter Dashboard & Surveillance System Contract

## 1. Overview & Architecture Split
The workspace has been factored into two distinct ROS 2 packages adhering to the project's responsibilities:
- **`surveillance_sim`**: Owns the simulation environment, physical SDF description, lighting, obstacles, actor kinematic animation clips, and Gazebo-to-ROS bridges.
- **`vision_perception`**: Owns multi-camera YOLO perception, ground projection, metric activity classification, goal pose emission, and annotated frame streaming.

## 2. Topic & Contract Specification

| Contract Topic | Message Type | Frequency / Behavior | Description |
| :--- | :--- | :--- | :--- |
| `/person_activity` | `std_msgs/msg/String` | 1–10 Hz (when visible) | Strictly `"Walking"`, `"Running"`, or `"Staying"` (case-sensitive). Publishes **nothing** when no person is visible. With multiple persons, publishes the most serious label (`Staying > Running > Walking`). |
| `/yolo/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | <= 10 fps | JPEG stream (quality ~70). Composed side-by-side: Overhead camera (left, 320x240) + Corner camera (right, 320x240) = 640x240 (at most 640x480). Draws bounding boxes, global track IDs, activity labels, and Cartographer map `(x, y)` coordinates. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Event-driven (once per episode) | Emitted once when a tracked person dwells continuously in `"Staying"` for >= 5.0 s. Frame is `"map"`, `z=0`, `w=1.0`. Enforces a 15.0 s global cooldown. Nudges to nearest free space within 0.5 m if `/map` is provided. |
| `/surveillance_camera/image_raw` | `sensor_msgs/msg/Image` | ~30 Hz | Raw overhead camera stream from room center `(0.0, 0.0, 12.2)` looking straight down. |
| `/corner_camera/image_raw` | `sensor_msgs/msg/Image` | ~10 Hz | Auxiliary corner camera stream from SW mount `(-7.0, -7.0, 2.3)`. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Latched (`TRANSIENT_LOCAL`) | Subscribed by `yolo_node` to verify goal pose free space (`cell == 0`). If map is absent, publishes unadjusted with a warning. |

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
