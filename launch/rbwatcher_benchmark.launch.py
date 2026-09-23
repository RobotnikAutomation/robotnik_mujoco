"""Launch one MuJoCo scene containing one to three namespaced RB-Watchers."""

from __future__ import annotations

import importlib.util
import os
import shutil
import tempfile
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


_RUNTIME_DIRS: list[Path] = []


def _as_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in ("true", "false"):
        raise RuntimeError(f"{name} must be true or false, got {value!r}")
    return normalized == "true"


def _cleanup(_context, *args, **kwargs):
    del args, kwargs
    for path in _RUNTIME_DIRS:
        shutil.rmtree(path, ignore_errors=True)
    _RUNTIME_DIRS.clear()
    return []


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    world = LaunchConfiguration("world").perform(context).strip().lower()
    gui = _as_bool(LaunchConfiguration("gui").perform(context), "gui")
    run_rviz = _as_bool(LaunchConfiguration("run_rviz").perform(context), "run_rviz")
    render_fps = float(LaunchConfiguration("render_fps").perform(context))
    if render_fps < 0.0 or render_fps > 1000.0:
        raise RuntimeError("render_fps must be between 0 and 1000")
    if num_robots not in (1, 2, 3):
        raise RuntimeError("num_robots must be 1, 2, or 3")
    if world == "demo":
        world = "simple"
    if world not in ("empty", "simple"):
        raise RuntimeError("world must be empty, simple, or demo")

    package_share = Path(get_package_share_directory("robotnik_mujoco"))
    package_prefix = package_share.parents[1]
    generator_path = package_prefix / "lib" / "robotnik_mujoco" / "generate_benchmark_assets.py"
    spec = importlib.util.spec_from_file_location("robotnik_mujoco_asset_generator", generator_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load asset generator from {generator_path}")
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)

    runtime_dir = Path(tempfile.mkdtemp(prefix="robotnik_mujoco_"))
    _RUNTIME_DIRS.append(runtime_dir)
    _scene_path, urdf_path, parameters_path = generator.generate(
        runtime_dir,
        num_robots,
        world,
        not gui,
        package_share / "models" / "rbwatcher",
        render_fps,
    )
    parameter_file = ParameterFile(str(parameters_path), allow_substs=False)
    description_topic = f"/mujoco/run_{uuid.uuid4().hex}/robot_description"

    description_publisher = Node(
        package="robotnik_mujoco",
        executable="robot_description_publisher.py",
        namespace="mujoco",
        output="both",
        parameters=[{"description_file": str(urdf_path), "topic_name": description_topic}],
    )
    state_publishers = []
    for index in range(1, num_robots + 1):
        name = f"rbwatcher_{index}"
        rsp_description = (runtime_dir / f"{name}_rsp.urdf").read_text(encoding="utf-8")
        state_publishers.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=name,
                output="both",
                parameters=[
                    {"robot_description": ParameterValue(rsp_description, value_type=str)},
                    {"use_sim_time": True},
                ],
                # The one controller manager owns a single joint-state stream
                # for all robots.  Every robot_state_publisher must consume it
                # explicitly; otherwise it subscribes to its private
                # /rbwatcher_N/joint_states topic and leaves wheel joints out
                # of the TF tree.
                remappings=[
                    ("/robot_description", f"/{name}/robot_description"),
                    ("joint_states", "/mujoco/joint_states"),
                ],
            )
        )
    control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="controller_manager",
        namespace="mujoco",
        emulate_tty=True,
        output="both",
        parameters=[{"use_sim_time": True}, parameter_file],
        remappings=[("robot_description", description_topic)],
    )
    state_bridge = Node(
        package="robotnik_mujoco",
        executable="mujoco_state_bridge.py",
        output="both",
        parameters=[{"num_robots": num_robots, "use_sim_time": True}],
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawn_joint_state_broadcaster",
            arguments=["joint_state_broadcaster", "--controller-manager", "/mujoco/controller_manager",
                       "--param-file", str(parameters_path)],
            output="both",
        )
    ]
    rviz_environment = {}
    if os.environ.get("SNAP"):
        # GUI applications launched from a snap-hosted terminal (for example
        # VS Code) otherwise inherit GTK module paths from the snap runtime.
        # Those modules are ABI-incompatible with Ubuntu 24.04's RViz.
        rviz_environment = {
            "GDK_PIXBUF_MODULEDIR": "",
            "GDK_PIXBUF_MODULE_FILE": "",
            "GIO_MODULE_DIR": "",
            "GSETTINGS_SCHEMA_DIR": "",
            "GTK_EXE_PREFIX": "",
            "GTK_IM_MODULE_FILE": "",
            "GTK_MODULES": "",
            "GTK_PATH": "",
            "LOCPATH": "",
            "XDG_DATA_HOME": str(Path.home() / ".local" / "share"),
            "XDG_DATA_DIRS": os.environ.get(
                "XDG_DATA_DIRS_VSCODE_SNAP_ORIG", "/usr/local/share:/usr/share"
            ),
        }
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=[
            "-d",
            str(Path(get_package_share_directory("robotnik_mujoco")) / "rviz" / "rbwatcher_benchmark.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        additional_env=rviz_environment,
        condition=IfCondition(str(run_rviz).lower()),
    )

    return [
        description_publisher,
        *state_publishers,
        control,
        state_bridge,
        *spawners,
        rviz,
        RegisterEventHandler(
            OnProcessExit(
                target_action=control,
                on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo controller manager exited"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="1", description="Number of robots (1..3)"),
            DeclareLaunchArgument("world", default_value="empty", description="empty, simple, or demo"),
            DeclareLaunchArgument("gui", default_value="true", description="Show the MuJoCo GUI"),
            DeclareLaunchArgument("run_rviz", default_value="false", description="Start RViz2"),
            DeclareLaunchArgument("render_fps", default_value="0", description="Native MuJoCo GUI FPS cap; 0 disables it"),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup)])),
            OpaqueFunction(function=_launch_setup),
        ]
    )
