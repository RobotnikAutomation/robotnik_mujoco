# robotnik_mujoco

A package to evaluate Robotnik robot dynamics in mujoco. The package links mujoco model to ros2 control

# NOTES: 
Model compile requires stl to be in the urdf folder
Have not been able to compile urdf with file:// nor package:// instead I have used fixed folder paths
Current controller uses diff_drive_base_controller (speed) instead of the robotnik_controllers one TBD)
Tested with mujoco 3.3.7 and ros 2 humble

# INSTALL:

-Create mujoco_ws ros 2 workspace
-Install mujoco 
-Install https://github.com/moveit/mujoco_ros2_control and compile
-Install https://github.com/RobotnikAutomation/robotnik_mujoco and compile


# USAGE:

>ros2 launch robotnik_mujoco rbsummit_world.launch.py
>ros2 launch rbsummit_world_rough_terrain.launch.py
>ros2 launch rbsummit_world_nist_arenas.launch.py

on a second terminal launch ps game pad or 

>ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
