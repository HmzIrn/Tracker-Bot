"""Test that cameras.yaml configuration does not drift from surveillance_world.sdf."""

import os
import xml.etree.ElementTree as ET
import pytest
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def find_sdf_file() -> str:
    """Attempt to locate surveillance_world.sdf."""
    candidates = []

    # Check relative to this file in src workspace
    test_dir = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.abspath(os.path.join(test_dir, "..", "..", "surveillance_sim", "worlds", "surveillance_world.sdf")))

    # Check installed package share
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory("surveillance_sim")
            candidates.append(os.path.join(share, "worlds", "surveillance_world.sdf"))
        except Exception:
            pass

    for path in candidates:
        if os.path.isfile(path):
            return path

    return ""


def find_yaml_file() -> str:
    """Attempt to locate cameras.yaml."""
    candidates = []
    test_dir = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.abspath(os.path.join(test_dir, "..", "config", "cameras.yaml")))

    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory("vision_perception")
            candidates.append(os.path.join(share, "config", "cameras.yaml"))
        except Exception:
            pass

    for path in candidates:
        if os.path.isfile(path):
            return path

    return ""


def test_camera_poses_match_sdf():
    """Verify that camera poses in cameras.yaml match surveillance_world.sdf poses."""
    sdf_path = find_sdf_file()
    if not sdf_path:
        pytest.skip("surveillance_world.sdf not found in workspace or install")

    yaml_path = find_yaml_file()
    if not yaml_path:
        pytest.skip("cameras.yaml not found in workspace or install")

    with open(yaml_path, "r") as f:
        cam_yaml = yaml.safe_load(f)

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # Find world element
    world = root.find("world") if root.tag != "world" else root
    assert world is not None, "World tag not found in SDF"

    sdf_poses = {}
    for model in world.findall("model"):
        m_name = model.get("name")
        pose_elem = model.find("pose")
        if pose_elem is not None and pose_elem.text:
            coords = [float(v) for v in pose_elem.text.strip().split()]
            sdf_poses[m_name] = coords

    for cam_name in ["overhead_camera", "corner_camera"]:
        assert cam_name in cam_yaml, f"Expected {cam_name} in cameras.yaml"
        assert cam_name in sdf_poses, f"Expected {cam_name} model in {sdf_path}"

        yaml_pose = [float(v) for v in cam_yaml[cam_name]["pose"]]
        sdf_pose = sdf_poses[cam_name]

        assert len(yaml_pose) == 6, f"{cam_name} pose in YAML should have 6 values"
        assert len(sdf_pose) == 6, f"{cam_name} pose in SDF should have 6 values"

        for i, (yp, sp) in enumerate(zip(yaml_pose, sdf_pose)):
            assert yp == pytest.approx(sp, abs=1e-3), (
                f"{cam_name} pose index {i} mismatch: YAML={yp} vs SDF={sp}"
            )
