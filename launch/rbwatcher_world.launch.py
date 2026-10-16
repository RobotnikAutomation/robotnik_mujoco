"""Compatibility entrypoint for the canonical one-robot RBWatcher scene."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("robotnik_mujoco"), "launch", "rbwatcher_benchmark.launch.py"]
                    )
                ),
                launch_arguments={
                    "world": "empty",
                    "num_robots": "1",
                    "gui": "true",
                    "run_rviz": "false",
                }.items(),
            )
        ]
    )
