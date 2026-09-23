"""Contract tests for generated RBWatcher benchmark assets."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = PACKAGE_ROOT / "scripts" / "generate_benchmark_assets.py"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "rbwatcher_benchmark.launch.py"
MODEL_DIR = PACKAGE_ROOT / "models" / "rbwatcher"
SPEC = importlib.util.spec_from_file_location("generate_benchmark_assets", GENERATOR_PATH)
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)


@pytest.mark.parametrize("robot_count", (1, 2, 3))
def test_scene_and_namespaced_sensor_contract(tmp_path, robot_count):
    scene_path, urdf_path, params_path = GENERATOR.generate(
        tmp_path, robot_count, "simple", True, MODEL_DIR
    )
    scene = ET.parse(scene_path).getroot()
    urdf = ET.parse(urdf_path).getroot()
    params = yaml.safe_load(params_path.read_text(encoding="utf-8"))
    plugins = params["/**"]["ros__parameters"]["mujoco_plugins"]

    robot_bodies = [
        body
        for body in scene.findall("./worldbody/body")
        if body.attrib["name"].startswith("rbwatcher_")
    ]
    assert len(robot_bodies) == robot_count
    assert len(scene.findall("./sensor/plugin")) == robot_count
    assert len(scene.findall("./actuator/velocity")) == 4 * robot_count
    assert {mesh.attrib["name"] for mesh in scene.findall("./asset/mesh")} == {
        name.removesuffix(".stl") for name in GENERATOR.MJCF_MESHES
    }
    assert urdf.find("./link[@name='world']") is not None

    for index in range(1, robot_count + 1):
        name = f"rbwatcher_{index}"
        body = scene.find(f"./worldbody/body[@name='{name}_base_link']")
        assert body is not None
        assert body.attrib["pos"].split()[2] == "0.11"
        assert body.find(f"./geom[@name='{name}_chassis_visual'][@mesh='rbwatcher_chassis']") is not None
        assert body.find(f"./geom[@name='{name}_top_structure_visual'][@mesh='rbwatcher_top_structure']") is not None
        assert body.find(f"./camera[@name='{name}_front_camera']").attrib["resolution"] == "1920 1080"
        assert body.find(f"./camera[@name='{name}_top_camera']").attrib["resolution"] == "1280 720"

        assert plugins[f"imu_{index}"]["topic"] == f"/{name}/imu/data"
        assert plugins[f"imu_{index}"]["publish_rate"] == 100.0
        assert plugins[f"base_velocity_{index}"]["cmd_vel_topic"] == f"/{name}/cmd_vel"
        assert plugins[f"base_velocity_{index}"]["cmd_timeout"] == 0.5
        assert plugins["camera"][f"{name}_front_camera"]["image_topic"] == (
            f"/{name}/front_rgbd_camera/color/image_raw"
        )
        assert plugins["camera"][f"{name}_front_camera"]["publish_depth"] is False
        assert plugins["camera"][f"{name}_top_camera"]["depth_topic"] == (
            f"/{name}/top_rgbd_camera/depth/image_raw"
        )
        assert plugins["lidar"][f"{name}_lidar"] == {
            "frame_name": f"{name}/top_3d_laser_link",
            "topic": f"/{name}/top_laser/points",
        }


def test_physical_dimensions_and_rates_match_reference_profile(tmp_path):
    scene_path, _, params_path = GENERATOR.generate(tmp_path, 1, "empty", False, MODEL_DIR)
    scene = ET.parse(scene_path).getroot()
    params = yaml.safe_load(params_path.read_text(encoding="utf-8"))
    body = scene.find("./worldbody/body[@name='rbwatcher_1_base_link']")
    assert scene.find("./option").attrib["timestep"] == "0.02"

    wheel = body.find("./body[@name='rbwatcher_1_front_left_wheel']")
    assert wheel.attrib["pos"] == "0.22795 0.226 0"
    assert wheel.find("./joint").attrib["axis"] == "0 1 0"
    assert wheel.findall("./geom")[1].attrib["size"] == "0.11 0.075"
    top_collision = body.find("./geom[@name='rbwatcher_1_top_structure_collision']")
    assert top_collision.attrib["pos"] == "-0.18 0 0.28"
    assert top_collision.attrib["size"] == "0.145 0.145 0.12"

    lidar = scene.find("./extension/plugin/instance[@name='rbwatcher_1_lidar_instance']")
    lidar_config = {item.attrib["key"]: item.attrib["value"] for item in lidar.findall("./config")}
    assert lidar_config["resolution"] == "1800 16"
    assert lidar_config["update_rate"] == "10.0"
    plugins = params["/**"]["ros__parameters"]["mujoco_plugins"]
    assert params["/mujoco/controller_manager"]["ros__parameters"]["update_rate"] == 50
    assert plugins["camera"]["camera_publish_rate"] == 25.0


def test_simple_world_is_the_common_walled_arena(tmp_path):
    scene_path, _, _ = GENERATOR.generate(tmp_path, 1, "simple", True, MODEL_DIR)
    scene = ET.parse(scene_path).getroot()
    warehouse = scene.find("./worldbody/body[@name='simple_warehouse']")
    assert warehouse is not None
    assert warehouse.attrib["pos"] == "-2 0 0"
    assert warehouse.attrib["euler"] == "0 0 1.57079632679"
    walls = {
        geom.attrib["name"]: geom
        for geom in warehouse.findall("./geom")
        if geom.attrib["name"].startswith("simple_")
    }

    assert set(walls) == {
        "simple_north_wall",
        "simple_south_wall",
        "simple_east_wall",
        "simple_west_wall",
        "simple_central_divider",
        "simple_northwest_corridor",
        "simple_north_central_corridor",
        "simple_northeast_corridor",
        "simple_west_room_divider",
        "simple_east_room_divider",
    }
    assert walls["simple_central_divider"].attrib["size"] == "8.5 0.1 1.5"
    assert walls["simple_west_room_divider"].attrib["size"] == "0.1 2.95 1.5"


def test_wheel_joint_stream_and_rviz_sensor_qos_contract():
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert '("joint_states", "/mujoco/joint_states")' in launch_source

    rviz = yaml.safe_load((PACKAGE_ROOT / "rviz" / "rbwatcher_benchmark.rviz").read_text())
    displays = {display["Name"]: display for display in rviz["Visualization Manager"]["Displays"]}
    for name in ("LiDAR", "Front RGB", "Top RGB", "Top depth"):
        topic = displays[name]["Topic"]
        assert topic["Depth"] == 5
        assert topic["Durability Policy"] == "Volatile"
        assert topic["History Policy"] == "Keep Last"
        assert topic["Reliability Policy"] == "Best Effort"


def test_rsp_owns_only_fixed_base_tree(tmp_path):
    GENERATOR.generate(tmp_path, 1, "empty", True, MODEL_DIR)
    rsp = ET.parse(tmp_path / "rbwatcher_1_rsp.urdf").getroot()
    joints = {joint.attrib["name"]: joint for joint in rsp.findall("./joint")}

    assert rsp.find("./link[@name='rbwatcher_1/odom']") is None
    assert "rbwatcher_1_floating_base_joint" not in joints
    base_joint = joints["rbwatcher_1_base_footprint_joint"]
    assert base_joint.find("./origin").attrib["xyz"] == "0 0 0.11"
    assert rsp.find("./link[@name='rbwatcher_1/top_3d_laser_link']") is not None
    assert rsp.find("./link[@name='rbwatcher_1/top_rgbd_camera_color_optical_frame']") is not None
