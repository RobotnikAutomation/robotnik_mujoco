#!/usr/bin/env python3
"""Generate one self-contained MuJoCo scene and matching ros2_control URDF."""

from __future__ import annotations

import argparse
from pathlib import Path
from xml.sax.saxutils import escape

import yaml


SPAWNS = ((0.0, 0.0), (0.0, 2.0), (0.0, -2.0))
BASE_LINK_HEIGHT = 0.11
WHEEL_RADIUS = 0.11
WHEEL_HALF_WIDTH = 0.075
WHEEL_X = 0.22795
WHEEL_Y = 0.226

REQUIRED_MESHES = (
    "antenna_ANN_MB.stl",
    "helios_16p.stl",
    "intel_d435.stl",
    "link_750_base.stl",
    "link_750_pan.stl",
    "link_750_tilt.stl",
    "mastil_gps.stl",
    "rbwatcher_chassis.stl",
    "rbwatcher_top_structure.stl",
    "rubber_wheel.stl",
    "vectornav_vn100.stl",
)
MJCF_MESHES = tuple(name for name in REQUIRED_MESHES if name != "intel_d435.stl")


def _validate_model_dir(model_dir: Path) -> Path:
    model_dir = model_dir.expanduser().resolve()
    missing = [name for name in REQUIRED_MESHES if not (model_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(f"RBWatcher mesh directory {model_dir} is missing: {missing}")
    return model_dir


def _mesh_assets(model_dir: Path) -> str:
    entries = []
    for filename in MJCF_MESHES:
        name = filename.removesuffix(".stl")
        scale = ' scale="0.001 0.001 0.001"' if filename == "link_750_base.stl" else ""
        entries.append(
            f'    <mesh name="{name}" file="{escape(str(model_dir / filename))}"{scale}/>'
        )
    return "\n".join(entries)


def _robot_mjcf(index: int) -> str:
    name = f"rbwatcher_{index}"
    x, y = SPAWNS[index - 1]
    return f"""
    <body name="{name}_base_link" pos="{x} {y} {BASE_LINK_HEIGHT}">
      <freejoint name="{name}_floating_base_joint"/>
      <inertial pos="0 0 0" mass="25" diaginertia="2.203333 2.141667 1.141667"/>

      <!-- Visual geometry comes from the production RBWatcher description.  Convex,
           low-cost primitives remain responsible for stable contacts. -->
      <geom name="{name}_chassis_visual" type="mesh" mesh="rbwatcher_chassis"
            contype="0" conaffinity="0" group="2" rgba="0.10 0.10 0.10 1"/>
      <geom name="{name}_chassis_collision" type="box" pos="-0.02 0 0.10"
            size="0.34 0.18 0.14" rgba="0.10 0.10 0.10 0"
            friction="0.9 0.02 0.002"/>
      <geom name="{name}_top_structure_visual" type="mesh" mesh="rbwatcher_top_structure"
            pos="-0.17024 0 0.27609" contype="0" conaffinity="0" group="2"
            rgba="0.50 0.50 0.50 1"/>
      <!-- Keep this conservative contact primitive below the Helios origin.
           A box surrounding the sensor makes every ray immediately hit its
           own robot and the LiDAR extension rightfully rejects those
           sub-minimum-range returns. -->
      <geom name="{name}_top_structure_collision" type="box" pos="-0.18 0 0.28"
            size="0.145 0.145 0.12" rgba="0.50 0.50 0.50 0"
            friction="0.8 0.02 0.002"/>

      <body name="{name}_front_left_wheel" pos="{WHEEL_X} {WHEEL_Y} 0">
        <joint name="{name}_front_left_wheel_joint" type="hinge" axis="0 1 0" damping="0.2"/>
        <inertial pos="0 0 0" mass="6.5" diaginertia="0.03185 0.039325 0.03185"/>
        <geom type="mesh" mesh="rubber_wheel" contype="0" conaffinity="0" group="2"
              rgba="0.035 0.035 0.035 1"/>
        <geom type="cylinder" size="{WHEEL_RADIUS} {WHEEL_HALF_WIDTH}"
              quat="0.7071068 0.7071068 0 0" rgba="0.05 0.05 0.05 0"
              friction="0.9 0.005 0.0001" condim="3"/>
      </body>
      <body name="{name}_front_right_wheel" pos="{WHEEL_X} -{WHEEL_Y} 0">
        <joint name="{name}_front_right_wheel_joint" type="hinge" axis="0 1 0" damping="0.2"/>
        <inertial pos="0 0 0" mass="6.5" diaginertia="0.03185 0.039325 0.03185"/>
        <geom type="mesh" mesh="rubber_wheel" contype="0" conaffinity="0" group="2"
              rgba="0.035 0.035 0.035 1"/>
        <geom type="cylinder" size="{WHEEL_RADIUS} {WHEEL_HALF_WIDTH}"
              quat="0.7071068 0.7071068 0 0" rgba="0.05 0.05 0.05 0"
              friction="0.9 0.005 0.0001" condim="3"/>
      </body>
      <body name="{name}_back_left_wheel" pos="-{WHEEL_X} {WHEEL_Y} 0">
        <joint name="{name}_back_left_wheel_joint" type="hinge" axis="0 1 0" damping="0.2"/>
        <inertial pos="0 0 0" mass="6.5" diaginertia="0.03185 0.039325 0.03185"/>
        <geom type="mesh" mesh="rubber_wheel" contype="0" conaffinity="0" group="2"
              rgba="0.035 0.035 0.035 1"/>
        <geom type="cylinder" size="{WHEEL_RADIUS} {WHEEL_HALF_WIDTH}"
              quat="0.7071068 0.7071068 0 0" rgba="0.05 0.05 0.05 0"
              friction="0.9 0.005 0.0001" condim="3"/>
      </body>
      <body name="{name}_back_right_wheel" pos="-{WHEEL_X} -{WHEEL_Y} 0">
        <joint name="{name}_back_right_wheel_joint" type="hinge" axis="0 1 0" damping="0.2"/>
        <inertial pos="0 0 0" mass="6.5" diaginertia="0.03185 0.039325 0.03185"/>
        <geom type="mesh" mesh="rubber_wheel" contype="0" conaffinity="0" group="2"
              rgba="0.035 0.035 0.035 1"/>
        <geom type="cylinder" size="{WHEEL_RADIUS} {WHEEL_HALF_WIDTH}"
              quat="0.7071068 0.7071068 0 0" rgba="0.05 0.05 0.05 0"
              friction="0.9 0.005 0.0001" condim="3"/>
      </body>

      <geom name="{name}_imu_visual" type="mesh" mesh="vectornav_vn100"
            pos="-0.03534 0.132 0.16109" contype="0" conaffinity="0" group="2"
            rgba="0.55 0.05 0.05 1"/>
      <geom name="{name}_lidar_visual" type="mesh" mesh="helios_16p"
            pos="-0.11924 0 0.38609" contype="0" conaffinity="0" group="2"
            rgba="0.18 0.18 0.18 1"/>
      <!-- intel_d435.stl has 231186 faces, above MuJoCo's 200000-face STL limit.
           Use the exact D435 envelope in MJCF; RViz still uses the packaged mesh. -->
      <geom name="{name}_front_camera_visual" type="box" size="0.012525 0.045 0.0125"
            pos="0.30881 0.0025 0.15243"
            contype="0" conaffinity="0" group="2" rgba="0.22 0.22 0.22 1"/>
      <geom name="{name}_ptz_base_visual" type="mesh" mesh="link_750_base"
            pos="-0.18808 -0.07 0.50929" contype="0" conaffinity="0" group="2"
            rgba="0.10 0.10 0.10 1"/>
      <geom name="{name}_ptz_pan_visual" type="mesh" mesh="link_750_pan"
            pos="-0.11808 0 0.58609" contype="0" conaffinity="0" group="2"
            rgba="0.10 0.10 0.10 1"/>
      <geom name="{name}_ptz_tilt_visual" type="mesh" mesh="link_750_tilt"
            pos="-0.11808 0 0.68809" euler="1.5707963 0 0"
            contype="0" conaffinity="0" group="2" rgba="0.10 0.10 0.10 1"/>
      <geom name="{name}_gps_mast_visual" type="mesh" mesh="mastil_gps"
            pos="-0.24824 0.095 0.39359" contype="0" conaffinity="0" group="2"
            rgba="0.10 0.10 0.10 1"/>
      <geom name="{name}_gps_antenna_visual" type="mesh" mesh="antenna_ANN_MB"
            pos="-0.24824 0.095 0.76359" contype="0" conaffinity="0" group="2"
            rgba="0.65 0.65 0.65 1"/>

      <site name="{name}_imu_site" pos="-0.04279 0.132 0.16989" size="0.01" rgba="0 1 0 1"/>
      <site name="{name}_lidar_site" pos="-0.11924 0 0.44959" euler="0 0 0" size="0.015"
            rgba="1 0.3 0 1"/>
      <camera name="{name}_front_camera" pos="0.30881 0.035 0.15243" fovy="69"
              euler="1.5707963 -1.5707963 0" resolution="1920 1080"/>
      <camera name="{name}_top_camera" pos="-0.06808 0 0.71309" fovy="45"
              euler="1.5707963 -1.5707963 0" resolution="1280 720"/>
    </body>"""


def _scene_xml(num_robots: int, world: str, model_dir: Path) -> str:
    lidar_instances = []
    sensors = []
    actuators = []
    contact_exclusions = []
    for index in range(1, num_robots + 1):
        name = f"rbwatcher_{index}"
        lidar_instances.append(
            f"""      <instance name="{name}_lidar_instance">
        <config key="resolution" value="1800 16"/>
        <config key="azimuth_range" value="-3.141592653589793 3.141592653589793"/>
        <config key="elevation_range" value="-0.261799 0.261799"/>
        <config key="max_range" value="100.0"/>
        <config key="min_range" value="0.1"/>
        <config key="update_rate" value="10.0"/>
        <config key="async" value="0"/>
      </instance>"""
        )
        sensors.append(
            f"""    <framequat name="{name}_imu_quat" objtype="site" objname="{name}_imu_site"/>
    <gyro name="{name}_imu_gyro" site="{name}_imu_site"/>
    <accelerometer name="{name}_imu_accel" site="{name}_imu_site"/>
    <plugin name="{name}_lidar" instance="{name}_lidar_instance"
            objtype="site" objname="{name}_lidar_site"/>"""
        )
        for wheel in ("front_left", "front_right", "back_left", "back_right"):
            joint = f"{name}_{wheel}_wheel_joint"
            actuators.append(
                f'    <velocity name="{joint}" joint="{joint}" kv="0.001" ctrlrange="-30 30"/>'
            )
            contact_exclusions.append(
                f'    <exclude body1="{name}_base_link" body2="{name}_{wheel}_wheel"/>'
            )

    obstacles = ""
    if world == "simple":
        # Reproduce robotnik_gazebo_ignition's warehouse_simple plan: a 20 x
        # 20 m, 3 m-high enclosure with a central divider, corridors, and
        # three upper rooms.  The body pose is the pose used by
        # lightweight_scene.world (x=-2, yaw=pi/2), so the layout matches the
        # common Gazebo benchmark rather than being merely a four-wall box.
        obstacles = """
    <body name="simple_warehouse" pos="-2 0 0" euler="0 0 1.57079632679">
      <geom name="simple_north_wall" type="box" pos="0 10.1 1.5" size="10.1 0.1 1.5"
            rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_south_wall" type="box" pos="0 -10.1 1.5" size="10.1 0.1 1.5"
            rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_east_wall" type="box" pos="10.1 0 1.5" size="0.1 10.1 1.5"
            rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_west_wall" type="box" pos="-10.1 0 1.5" size="0.1 10.1 1.5"
            rgba="0.65 0.65 0.65 1"/>

      <geom name="simple_central_divider" type="box" pos="-1.5 0 1.5" size="8.5 0.1 1.5"
            rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_northwest_corridor" type="box" pos="-5.11665 4.0 1.5"
            size="1.88335 0.1 1.5" rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_north_central_corridor" type="box" pos="1.55 4.0 1.5"
            size="1.88335 0.1 1.5" rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_northeast_corridor" type="box" pos="8.16665 4.0 1.5"
            size="1.83335 0.1 1.5" rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_west_room_divider" type="box" pos="-3.3333 7.05 1.5"
            size="0.1 2.95 1.5" rgba="0.65 0.65 0.65 1"/>
      <geom name="simple_east_room_divider" type="box" pos="3.3333 7.05 1.5"
            size="0.1 2.95 1.5" rgba="0.65 0.65 0.65 1"/>
    </body>"""

    robots = "\n".join(_robot_mjcf(i) for i in range(1, num_robots + 1))
    return f"""<?xml version="1.0"?>
<mujoco model="rbwatcher_benchmark_{world}_{num_robots}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.02" gravity="0 0 -9.81" integrator="implicitfast"/>
  <size memory="512M"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.25 0.25 0.25" specular="0.1 0.1 0.1"/>
    <global azimuth="130" elevation="-25"/>
  </visual>
  <extension>
    <plugin plugin="mujoco.plugin.lidar">
{chr(10).join(lidar_instances)}
    </plugin>
  </extension>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.35 0.5 0.7" rgb2="0.02 0.02 0.04"
             width="512" height="3072"/>
    <texture type="2d" name="ground_tex" builtin="checker" rgb1="0.25 0.28 0.30"
             rgb2="0.16 0.18 0.20" width="512" height="512"/>
    <material name="ground" texture="ground_tex" texrepeat="10 10" texuniform="true"/>
{_mesh_assets(model_dir)}
  </asset>
  <worldbody>
    <light pos="0 0 6" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="ground"
          friction="1.0 0.005 0.0001" condim="3"/>
{obstacles}
{robots}
  </worldbody>
  <sensor>
{chr(10).join(sensors)}
  </sensor>
  <contact>
{chr(10).join(contact_exclusions)}
  </contact>
  <actuator>
{chr(10).join(actuators)}
  </actuator>
</mujoco>
"""


def _robot_urdf(index: int, standalone: bool = False) -> str:
    name = f"rbwatcher_{index}"
    root = "" if standalone else f"""
  <link name="{name}/odom"/>
  <joint name="world_to_{name}_odom" type="fixed">
    <parent link="world"/><child link="{name}/odom"/><origin xyz="0 0 0"/>
  </joint>"""
    floating_joint = "" if standalone else f"""
  <joint name="{name}_floating_base_joint" type="floating">
    <parent link="{name}/odom"/><child link="{name}/base_footprint"/>
  </joint>"""
    wheels = []
    for wheel, xyz in (
        ("front_left", f"{WHEEL_X} {WHEEL_Y} 0"),
        ("front_right", f"{WHEEL_X} -{WHEEL_Y} 0"),
        ("back_left", f"-{WHEEL_X} {WHEEL_Y} 0"),
        ("back_right", f"-{WHEEL_X} -{WHEEL_Y} 0"),
    ):
        wheels.append(
            f"""
  <link name="{name}/{wheel}_wheel">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/rubber_wheel.stl"/></geometry>
      <material name="dark"><color rgba="0.05 0.05 0.05 1"/></material></visual>
  </link>
  <joint name="{name}_{wheel}_wheel_joint" type="continuous">
    <parent link="{name}/base_link"/><child link="{name}/{wheel}_wheel"/>
    <origin xyz="{xyz}"/><axis xyz="0 1 0"/>
  </joint>"""
        )
    return f"""{root}
  <link name="{name}/base_footprint"/>
  <!-- odom->base_footprint is dynamic and comes from MuJoCo via mujoco_state_bridge. -->
{floating_joint}
  <link name="{name}/base_link">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/rbwatcher_chassis.stl"/></geometry>
      <material name="darkgrey"><color rgba="0.10 0.10 0.10 1"/></material></visual>
  </link>
  <joint name="{name}_base_footprint_joint" type="fixed">
    <parent link="{name}/base_footprint"/><child link="{name}/base_link"/>
    <origin xyz="0 0 {BASE_LINK_HEIGHT}"/>
  </joint>
{''.join(wheels)}
  <link name="{name}/top_structure_link">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/rbwatcher_top_structure.stl"/></geometry>
      <material name="grey"><color rgba="0.50 0.50 0.50 1"/></material></visual>
  </link>
  <joint name="{name}_top_structure_joint" type="fixed">
    <parent link="{name}/base_link"/><child link="{name}/top_structure_link"/>
    <origin xyz="-0.17024 0 0.27609"/>
  </joint>
  <link name="{name}/imu_base_link">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/vectornav_vn100.stl"/></geometry>
      <material name="imu_red"><color rgba="0.55 0.05 0.05 1"/></material></visual>
  </link>
  <joint name="{name}_imu_base_joint" type="fixed">
    <parent link="{name}/base_link"/><child link="{name}/imu_base_link"/>
    <origin xyz="-0.03534 0.132 0.16109"/>
  </joint>
  <link name="{name}/imu_link"/>
  <joint name="{name}_imu_joint" type="fixed">
    <parent link="{name}/imu_base_link"/><child link="{name}/imu_link"/>
    <origin xyz="-0.00745 0 0.0088"/>
  </joint>
  <link name="{name}/top_3d_laser_base_link">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/helios_16p.stl"/></geometry>
      <material name="lidar_grey"><color rgba="0.18 0.18 0.18 1"/></material></visual>
  </link>
  <joint name="{name}_top_3d_laser_base_joint" type="fixed">
    <parent link="{name}/base_link"/><child link="{name}/top_3d_laser_base_link"/>
    <origin xyz="-0.11924 0 0.38609"/>
  </joint>
  <link name="{name}/top_3d_laser_link"/>
  <joint name="{name}_top_3d_laser_joint" type="fixed">
    <parent link="{name}/top_3d_laser_base_link"/><child link="{name}/top_3d_laser_link"/>
    <origin xyz="0 0 0.0635"/>
  </joint>
  <link name="{name}/front_rgbd_camera_base_link"/>
  <joint name="{name}_front_rgbd_camera_base_joint" type="fixed">
    <parent link="{name}/base_link"/><child link="{name}/front_rgbd_camera_base_link"/>
    <origin xyz="0.29881 0 0.15243"/>
  </joint>
  <link name="{name}/front_rgbd_camera_link">
    <visual><origin xyz="0.0149 -0.0175 0" rpy="1.5707963 0 1.5707963"/>
      <geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/intel_d435.stl"/></geometry>
      <material name="camera_grey"><color rgba="0.22 0.22 0.22 1"/></material></visual>
  </link>
  <joint name="{name}_front_rgbd_camera_joint" type="fixed">
    <parent link="{name}/front_rgbd_camera_base_link"/><child link="{name}/front_rgbd_camera_link"/>
    <origin xyz="0.01 0.02 0"/>
  </joint>
  <link name="{name}/front_rgbd_camera_color_frame"/>
  <joint name="{name}_front_camera_color_joint" type="fixed">
    <parent link="{name}/front_rgbd_camera_link"/><child link="{name}/front_rgbd_camera_color_frame"/>
    <origin xyz="0 0.015 0"/>
  </joint>
  <link name="{name}/front_rgbd_camera_color_optical_frame"/>
  <joint name="{name}_front_camera_optical_joint" type="fixed">
    <parent link="{name}/front_rgbd_camera_color_frame"/><child link="{name}/front_rgbd_camera_color_optical_frame"/>
    <origin rpy="-1.5707963 0 -1.5707963"/>
  </joint>
  <link name="{name}/top_ptz_camera_base_link">
    <visual><origin xyz="-0.07 -0.07 -0.045"/>
      <geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/link_750_base.stl" scale="0.001 0.001 0.001"/></geometry>
      <material name="ptz_dark"><color rgba="0.10 0.10 0.10 1"/></material></visual>
  </link>
  <joint name="{name}_top_ptz_camera_base_joint" type="fixed">
    <parent link="{name}/base_link"/><child link="{name}/top_ptz_camera_base_link"/>
    <origin xyz="-0.11808 0 0.55429"/>
  </joint>
  <link name="{name}/top_ptz_camera_pan_link">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/link_750_pan.stl"/></geometry>
      <material name="ptz_pan_dark"><color rgba="0.10 0.10 0.10 1"/></material></visual>
  </link>
  <joint name="{name}_top_ptz_camera_pan_joint" type="fixed">
    <parent link="{name}/top_ptz_camera_base_link"/><child link="{name}/top_ptz_camera_pan_link"/>
    <origin xyz="0 0 0.0318"/>
  </joint>
  <link name="{name}/top_ptz_camera_tilt_link">
    <visual><origin rpy="1.5707963 0 0"/>
      <geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/link_750_tilt.stl"/></geometry>
      <material name="ptz_tilt_dark"><color rgba="0.10 0.10 0.10 1"/></material></visual>
  </link>
  <joint name="{name}_top_ptz_camera_tilt_joint" type="fixed">
    <parent link="{name}/top_ptz_camera_pan_link"/><child link="{name}/top_ptz_camera_tilt_link"/>
    <origin xyz="0 0 0.102" rpy="1.5707963 0 0"/>
  </joint>
  <link name="{name}/top_ptz_camera_frame_link"/>
  <joint name="{name}_top_ptz_camera_frame_joint" type="fixed">
    <parent link="{name}/top_ptz_camera_tilt_link"/><child link="{name}/top_ptz_camera_frame_link"/>
    <origin xyz="0.05 0.025 0" rpy="-1.5707963 0 0"/>
  </joint>
  <link name="{name}/top_ptz_camera_optical_frame_link"/>
  <joint name="{name}_top_ptz_camera_optical_joint" type="fixed">
    <parent link="{name}/top_ptz_camera_frame_link"/><child link="{name}/top_ptz_camera_optical_frame_link"/>
    <origin rpy="-1.5707963 0 -1.5707963"/>
  </joint>
  <link name="{name}/top_rgbd_camera_color_optical_frame"/>
  <joint name="{name}_top_camera_canonical_joint" type="fixed">
    <parent link="{name}/top_ptz_camera_optical_frame_link"/><child link="{name}/top_rgbd_camera_color_optical_frame"/>
  </joint>
  <link name="{name}/gps_mast_base_link">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/mastil_gps.stl"/></geometry>
      <material name="gps_mast_dark"><color rgba="0.10 0.10 0.10 1"/></material></visual>
  </link>
  <joint name="{name}_gps_mast_joint" type="fixed">
    <parent link="{name}/base_link"/><child link="{name}/gps_mast_base_link"/>
    <origin xyz="-0.24824 0.095 0.39359"/>
  </joint>
  <link name="{name}/gps_base_link">
    <visual><geometry><mesh filename="package://robotnik_mujoco/models/rbwatcher/antenna_ANN_MB.stl"/></geometry>
      <material name="gps_grey"><color rgba="0.65 0.65 0.65 1"/></material></visual>
  </link>
  <joint name="{name}_gps_base_joint" type="fixed">
    <parent link="{name}/base_link"/><child link="{name}/gps_base_link"/>
    <origin xyz="-0.24824 0.095 0.76359"/>
  </joint>"""


def _control_urdf(
    num_robots: int, scene_path: Path, headless: bool, render_fps: float = 0.0
) -> str:
    robot_blocks = "\n".join(_robot_urdf(i) for i in range(1, num_robots + 1))
    interfaces = []
    for index in range(1, num_robots + 1):
        name = f"rbwatcher_{index}"
        for wheel in ("front_left", "front_right", "back_left", "back_right"):
            interfaces.append(
                f"""    <joint name="{name}_{wheel}_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/><state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>"""
            )
        interfaces.append(
            f"""    <sensor name="{name}_imu">
      <param name="mujoco_type">imu</param>
      <param name="mujoco_sensor_name">{name}_imu</param>
      <state_interface name="orientation.x"/><state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/><state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/><state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/><state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
    </sensor>"""
        )
    return f"""<?xml version="1.0"?>
<robot name="rbwatcher_mujoco_{num_robots}">
  <link name="world"/>
{robot_blocks}
  <ros2_control name="MujocoSystem" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">{escape(str(scene_path))}</param>
      <param name="headless">{str(headless).lower()}</param>
      <param name="sim_speed_factor">1.0</param>
      <param name="render_fps">{render_fps}</param>
      <param name="odom_free_joint_name">__disabled__</param>
    </hardware>
{chr(10).join(interfaces)}
  </ros2_control>
</robot>
"""


def _parameters(num_robots: int, headless: bool) -> dict:
    params: dict = {
        "/mujoco/controller_manager": {
            "ros__parameters": {
                "update_rate": 50,
                "use_sim_time": True,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
            }
        },
        "/mujoco/joint_state_broadcaster": {
            "ros__parameters": {"use_sim_time": True, "use_urdf_to_filter": True}
        },
        "/**": {"ros__parameters": {"mujoco_plugins": {}}},
    }
    plugins = params["/**"]["ros__parameters"]["mujoco_plugins"]
    plugins["camera"] = {
        "type": "mujoco_ros2_control_plugins/CameraPlugin",
        "camera_publish_rate": 25.0,
        "force_egl": headless,
        "defer_glfw_cleanup": not headless,
    }
    plugins["lidar"] = {"type": "mujoco_ros2_control_plugins/Mujoco3dLidarPlugin"}
    plugins["free_joint_states"] = {
        "type": "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin",
        "frame_id": "",
        "body_names": [f"rbwatcher_{i}_base_link" for i in range(1, num_robots + 1)],
        "topic": "/mujoco/free_joint_states",
        "publish_rate": 100.0,
    }

    for index in range(1, num_robots + 1):
        name = f"rbwatcher_{index}"
        plugins[f"imu_{index}"] = {
            "type": "mujoco_ros2_control_plugins/ImuPlugin",
            "sensor_name": f"{name}_imu",
            "frame_name": f"{name}/imu_link",
            "topic": f"/{name}/imu/data",
            "publish_rate": 100.0,
        }
        plugins[f"base_velocity_{index}"] = {
            "type": "mujoco_ros2_control_plugins/BaseVelocityPlugin",
            "body": f"{name}_base_link",
            "cmd_vel_topic": f"/{name}/cmd_vel",
            "use_stamped_twist": False,
            "max_linear_velocity": 2.0,
            "max_yaw_rate": 2.0,
            "cmd_timeout": 0.5,
        }
        plugins["camera"][f"{name}_front_camera"] = {
            "policy": "streaming",
            "publish_depth": False,
            "frame_name": f"{name}/front_rgbd_camera_color_optical_frame",
            "info_topic": f"/{name}/front_rgbd_camera/color/camera_info",
            "image_topic": f"/{name}/front_rgbd_camera/color/image_raw",
            "depth_topic": f"/{name}/front_rgbd_camera/depth/image_raw",
        }
        plugins["camera"][f"{name}_top_camera"] = {
            "policy": "streaming",
            "frame_name": f"{name}/top_rgbd_camera_color_optical_frame",
            "info_topic": f"/{name}/top_rgbd_camera/color/camera_info",
            "image_topic": f"/{name}/top_rgbd_camera/color/image_raw",
            "depth_topic": f"/{name}/top_rgbd_camera/depth/image_raw",
        }
        plugins["lidar"][f"{name}_lidar"] = {
            "frame_name": f"{name}/top_3d_laser_link",
            "topic": f"/{name}/top_laser/points",
        }
    return params


def generate(
    output_dir: Path,
    num_robots: int,
    world: str,
    headless: bool,
    model_dir: Path | None = None,
    render_fps: float = 0.0,
) -> tuple[Path, Path, Path]:
    if num_robots not in (1, 2, 3):
        raise ValueError("num_robots must be 1, 2, or 3")
    if world == "demo":
        world = "simple"
    if world not in ("empty", "simple"):
        raise ValueError("world must be empty, simple, or demo")
    if model_dir is None:
        model_dir = Path(__file__).resolve().parents[1] / "models" / "rbwatcher"
    model_dir = _validate_model_dir(model_dir)
    if render_fps < 0.0 or render_fps > 1000.0:
        raise ValueError("render_fps must be between 0 and 1000")
    output_dir.mkdir(parents=True, exist_ok=True)
    scene_path = output_dir / "scene.xml"
    urdf_path = output_dir / "rbwatcher.urdf"
    params_path = output_dir / "controllers.yaml"
    scene_path.write_text(_scene_xml(num_robots, world, model_dir), encoding="utf-8")
    urdf_path.write_text(
        _control_urdf(num_robots, scene_path, headless, render_fps), encoding="utf-8"
    )
    params_path.write_text(yaml.safe_dump(_parameters(num_robots, headless), sort_keys=False), encoding="utf-8")
    for index in range(1, num_robots + 1):
        rsp_path = output_dir / f"rbwatcher_{index}_rsp.urdf"
        rsp_path.write_text(
            '<?xml version="1.0"?>\n'
            f'<robot name="rbwatcher_{index}">\n'
            f'{_robot_urdf(index, standalone=True)}\n'
            '</robot>\n',
            encoding="utf-8",
        )
    return scene_path, urdf_path, params_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--num-robots", type=int, choices=(1, 2, 3), required=True)
    parser.add_argument("--world", choices=("empty", "simple", "demo"), required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-fps", type=float, default=0.0, help="Native GUI FPS cap; 0 disables it")
    parser.add_argument(
        "--model-dir",
        type=Path,
        help="Directory containing the installed RBWatcher STL meshes",
    )
    args = parser.parse_args()
    for path in generate(
        args.output_dir,
        args.num_robots,
        args.world,
        args.headless,
        args.model_dir,
        args.render_fps,
    ):
        print(path)


if __name__ == "__main__":
    main()
