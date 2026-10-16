#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [empty|simple|demo] [1|2|3] [rbwatcher] [true|false] [true|false]" >&2
  echo "       arguments: world num_robots robot_model run_rviz gui" >&2
}

if [[ $# -ne 5 ]]; then
  usage
  exit 64
fi

world=$1
num_robots=$2
robot_model=$3
run_rviz=$4
gui=$5

case "$world" in empty|simple) ;; demo) world=simple ;; *) usage; exit 64 ;; esac
case "$num_robots" in 1|2|3) ;; *) usage; exit 64 ;; esac
[[ "$robot_model" == "rbwatcher" ]] || { usage; exit 64; }
case "$run_rviz" in true|false) ;; *) usage; exit 64 ;; esac
case "$gui" in true|false) ;; *) usage; exit 64 ;; esac

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "robotnik_mujoco requires ROS_DISTRO=jazzy; source /opt/ros/jazzy/setup.bash first." >&2
  exit 69
fi
if ! ros2 pkg prefix robotnik_mujoco >/dev/null 2>&1; then
  echo "robotnik_mujoco is not in the current ROS overlay." >&2
  exit 69
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/robotnik_mujoco_logs}"
export ROS_HOME="${ROS_HOME:-/tmp/robotnik_mujoco_ros_home}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"

launch_pid=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$launch_pid" ]] && kill -0 -- "-$launch_pid" 2>/dev/null; then
    # The launch process is a session leader. Signal the complete, precisely
    # scoped session so no child can outlive it; ROS nodes treat SIGINT as an
    # orderly shutdown request.
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in {1..150}; do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
      kill -TERM -- "-$launch_pid" 2>/dev/null || true
      for _ in {1..50}; do
        kill -0 -- "-$launch_pid" 2>/dev/null || break
        sleep 0.1
      done
    fi
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
      kill -KILL -- "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

setsid ros2 launch robotnik_mujoco rbwatcher_benchmark.launch.py \
  world:="$world" num_robots:="$num_robots" gui:="$gui" run_rviz:="$run_rviz" &
launch_pid=$!

set +e
wait "$launch_pid"
status=$?
set -e
launch_pid=""
trap - INT TERM EXIT
exit "$status"
