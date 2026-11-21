"""Validate actor timing, waypoint speeds, room clearance, and camera frustum."""

import math
import os
import xml.etree.ElementTree as ET

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def load_actor_script():
    candidates = [
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "surveillance_sim",
                "worlds",
                "surveillance_world.sdf",
            )
        )
    ]
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory("surveillance_sim")
            candidates.append(os.path.join(share, "worlds", "surveillance_world.sdf"))
        except Exception:
            pass

    sdf_path = next((path for path in candidates if os.path.isfile(path)), None)
    assert sdf_path is not None, "surveillance_world.sdf not found in source or install"
    root = ET.parse(sdf_path).getroot()
    actor = root.find(".//actor[@name='human_subject']")
    assert actor is not None
    script = actor.find("script")
    assert script is not None
    return root, script


def read_waypoints(trajectory):
    points = []
    for waypoint in trajectory.findall("waypoint"):
        time_sec = float(waypoint.findtext("time"))
        pose = [float(value) for value in waypoint.findtext("pose").split()]
        points.append((time_sec, pose))
    return points


def segment_speeds(points):
    speeds = []
    for (time_a, pose_a), (time_b, pose_b) in zip(points, points[1:]):
        distance = math.hypot(pose_b[0] - pose_a[0], pose_b[1] - pose_a[1])
        duration = time_b - time_a
        if distance > 0.0:
            speeds.append(distance / duration)
    return speeds


def test_actor_cycle_keeps_twelve_second_idle_and_returns_to_origin():
    _, script = load_actor_script()
    idle = next(t for t in script.findall("trajectory") if t.get("type") == "idle")
    points = read_waypoints(idle)

    assert points[0][0] == 0.0
    assert points[-1][0] == 12.0
    assert points[0][1][:2] == [0.0, 0.0]
    assert points[-1][1][:2] == [0.0, 0.0]

    trajectories = script.findall("trajectory")
    final_waypoint = max(
        (point for trajectory in trajectories for point in read_waypoints(trajectory)),
        key=lambda point: point[0],
    )
    assert final_waypoint[0] == 27.0
    assert final_waypoint[1][:2] == [0.0, 0.0]


def test_actor_walk_and_run_waypoint_speeds_match_thresholds():
    _, script = load_actor_script()
    trajectories = script.findall("trajectory")
    walk_speeds = [
        speed
        for trajectory in trajectories
        if trajectory.get("type") == "walk"
        for speed in segment_speeds(read_waypoints(trajectory))
    ]
    run = next(t for t in trajectories if t.get("type") == "run")
    run_speeds = segment_speeds(read_waypoints(run))
    run_points = read_waypoints(run)
    run_average = sum(run_speeds[i] * (run_points[i + 1][0] - run_points[i][0])
                      for i in range(len(run_speeds))) / (
        run_points[-1][0] - run_points[0][0]
    )

    assert walk_speeds
    assert all(0.8 <= speed <= 1.3 for speed in walk_speeds)
    assert run_speeds
    assert all(speed >= 3.0 for speed in run_speeds)
    assert run_average >= 3.0


def point_to_rotated_box_distance(point, center, size, yaw):
    dx, dy = point[0] - center[0], point[1] - center[1]
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    outside_x = max(abs(local_x) - size[0] / 2.0, 0.0)
    outside_y = max(abs(local_y) - size[1] / 2.0, 0.0)
    return math.hypot(outside_x, outside_y)


def test_actor_path_is_inside_camera_frustum_and_clear_of_obstacles():
    root, script = load_actor_script()
    camera = root.find(".//model[@name='corner_camera']")
    assert camera is not None
    camera_pose = [float(value) for value in camera.findtext("pose").split()]
    sensor_camera = camera.find(".//sensor[@type='camera']/camera")
    assert sensor_camera is not None
    hfov = float(sensor_camera.findtext("horizontal_fov"))
    width = int(sensor_camera.findtext("image/width"))
    height = int(sensor_camera.findtext("image/height"))
    vertical_half_fov = math.atan((height / width) * math.tan(hfov / 2.0))

    timed_waypoints = sorted(
        (time_sec, pose)
        for trajectory in script.findall("trajectory")
        for time_sec, pose in read_waypoints(trajectory)
    )
    path_points = []
    for (time_a, pose_a), (time_b, pose_b) in zip(
        timed_waypoints, timed_waypoints[1:]
    ):
        intervals = max(1, math.ceil((time_b - time_a) * 10))
        for step in range(intervals):
            ratio = step / intervals
            path_points.append(
                tuple(
                    pose_a[axis] + ratio * (pose_b[axis] - pose_a[axis])
                    for axis in range(3)
                )
            )
    path_points.append(tuple(timed_waypoints[-1][1][:3]))
    obstacles = (
        ((-3.5, 3.5), (0.8, 0.8), 0.0),
        ((3.5, 3.5), (0.8, 0.8), 0.0),
        ((-4.0, -4.5), (1.2, 1.2), 0.2),
        ((4.5, -4.0), (1.5, 1.0), -0.15),
    )

    for x, y, z in path_points:
        assert -7.5 <= x <= 7.5 and -7.5 <= y <= 7.5
        point = (x, y)
        assert min(
            point_to_rotated_box_distance(point, center, size, yaw)
            for center, size, yaw in obstacles
        ) >= 0.3

        dx, dy = x - camera_pose[0], y - camera_pose[1]
        azimuth = math.atan2(dy, dx)
        azimuth_error = (azimuth - camera_pose[5] + math.pi) % (2 * math.pi) - math.pi
        horizontal_distance = math.hypot(dx, dy)
        depression = math.atan2(camera_pose[2] - z, horizontal_distance)
        vertical_error = abs(depression - camera_pose[4])
        assert abs(azimuth_error) <= hfov / 2.0
        assert vertical_error <= vertical_half_fov
