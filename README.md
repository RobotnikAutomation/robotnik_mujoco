# robotnik_mujoco

MuJoCo backend for the Robotnik RB-Watcher benchmark. The supported reference platform is
**Ubuntu 24.04 (Noble) with ROS 2 Jazzy**. Humble-era launch files, packages and APIs are not
supported by the benchmark entrypoint.

## Stack version

This workspace is validated with `ros-controls/mujoco_ros2_control` 0.1.2, branch `main`, commit
`58f465b1b4634a6f366e71919189574f5d95c1e8`, and the Jazzy `mujoco_vendor` package. The local
checkout supplies camera, native ray-cast 3D LiDAR, free-joint and base-velocity plugins. A small
IMU plugin in that checkout publishes MuJoCo's real `framequat`, `gyro` and `accelerometer`
measurements with sensor-data QoS; it does not synthesize sensor messages from wall time.

## Clean build

```bash
cd /home/robotnik/jlgalan_dev/mujoco_ws
source /opt/ros/jazzy/setup.bash
rosdep check --from-paths src --ignore-src --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

The first source build of `mujoco_ros2_control` may fetch `lodepng`, because the Jazzy
`mujoco_vendor` CMake export does not currently expose its bundled version. All remaining ROS
dependencies resolve through `rosdep` on Noble/Jazzy.

## Standalone launch

```bash
ros2 launch robotnik_mujoco rbwatcher_benchmark.launch.py \
  world:=empty num_robots:=1 gui:=false run_rviz:=false
```

The native MuJoCo window can be capped without MangoHud. Set `render_fps:=60`
for a 60 FPS cap, or leave it at `0` to keep the window uncapped:

```bash
ros2 launch robotnik_mujoco rbwatcher_benchmark.launch.py \
  world:=simple num_robots:=3 gui:=true run_rviz:=false render_fps:=60
```

This pacing applies only to the onscreen `simulate` UI loop. Physics, `/clock`,
RTF, and offscreen ROS camera rendering remain independent.

Arguments:

- `num_robots`: `1`, `2`, or `3`.
- `world`: `empty`, `simple`, or the alias `demo` (mapped to `simple`).  The
  simple world is the common 20 x 20 m, 3 m-high warehouse layout used for the
  lightweight benchmark, including its interior corridors and rooms; it has no
  ramp or unrelated interior props.

RViz2 in the supported Jazzy installation has no native `Imu` display plugin.
The benchmark RViz profile therefore shows the robot, TF, LiDAR, and camera
streams; inspect `/rbwatcher_N/imu/data` with `ros2 topic echo` or a dedicated
IMU visualization plugin when needed.
- `gui`: controls the MuJoCo window; cameras still render through an offscreen context when false.
- `run_rviz`: starts one RViz process; keep it false when another launcher owns RViz.

All robots live in one generated MJCF scene and share one physics clock. Runtime files are created
in a private directory under `/tmp` and removed on shutdown. The visual model uses the RBWatcher
STL assets installed by this package (chassis, upper structure, wheels, VectorNav, Helios, D435,
PTZ and GPS). Stable primitive collision shapes are kept separate from those high-polygon meshes.
Cameras and the 16 x 1800 LiDAR are rendered or ray-cast by MuJoCo plugins.

The wrapper used by automation is:

```bash
$(ros2 pkg prefix robotnik_mujoco)/share/robotnik_mujoco/scripts/benchmark_mujoco.sh \
  empty 1 rbwatcher false false
```

It validates arguments, launches ROS in a separate process session and forwards INT/TERM to the
whole session. It also places ROS logs and controller-spawner locks under `/tmp` by default.

## Sensor and control checks

```bash
ros2 topic hz /rbwatcher_1/front_rgbd_camera/color/image_raw
ros2 topic hz /rbwatcher_1/top_rgbd_camera/color/image_raw
ros2 topic hz /rbwatcher_1/top_rgbd_camera/depth/image_raw
ros2 topic hz /rbwatcher_1/top_laser/points
ros2 topic hz /rbwatcher_1/imu/data
ros2 topic echo --once /clock
ros2 topic echo --once /rbwatcher_1/odom
ros2 topic pub --rate 10 /rbwatcher_1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.1}}'
ros2 run tf2_ros tf2_echo world rbwatcher_1/base_footprint
```

Canonical streams use best-effort, volatile, keep-last QoS with depth 5. Camera timestamps advance
at 25 Hz in simulation time, LiDAR at 10 Hz, IMU at 100 Hz, and `/clock` comes directly from each
MuJoCo physics step. On a machine whose renderer cannot maintain real time, wall-clock `topic hz`
is expected to fall with RTF while the simulated timestamp cadence stays canonical.

## Overlay order

For use with the benchmark runner:

```bash
source /opt/ros/jazzy/setup.bash
source /home/robotnik/jlgalan_dev/mujoco_ws/install/setup.bash
source /home/robotnik/jlgalan_dev/benchmark_ws/install/setup.bash
```

Run the local contract tests with:

```bash
cd /home/robotnik/jlgalan_dev/mujoco_ws/src/robotnik_mujoco
python3 -m pytest -q tests/test_benchmark_assets.py
```

## Deliberate simplifications

- `cmd_vel` is enforced by the MuJoCo base-velocity plugin with a 0.5 s timeout. Wheels, contacts
  and ros2_control joint interfaces are present, but this benchmark model does not reproduce the
  production motor controllers or estimate odometry from wheel slip.
- The PTZ assembly is fixed at its zero pan/tilt pose. It publishes the canonical
  `top_rgbd_camera` streams rather than Gazebo's backend-specific `top_ptz_rgbd_camera` names.
- Collision geometry is a conservative set of boxes and cylinders, not the visual triangle mesh.
- MuJoCo's STL loader rejects the 231,186-face D435 asset, so its MJCF visual uses the real camera
  envelope; the URDF/RViz model continues to use the detailed packaged mesh.
- Only the benchmark `empty` and `simple` worlds are generated. GPS hardware is visualized but is
  not part of the canonical sensor workload and therefore has no ROS publisher.
- Headless cameras require working EGL/OpenGL drivers; when rendering cannot maintain real time,
  sensor wall rates fall with RTF while simulation-time periods remain fixed.
