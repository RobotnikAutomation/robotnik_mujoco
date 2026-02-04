# robotnik_mujoco

A package to evaluate Robotnik robot dynamics in mujoco. The package links mujoco model to ros2 control

# NOTES: 
Model compile requires stl to be in the urdf folder  
Have not been able to compile urdf with file:// nor package:// instead I have used fixed folder paths  
Usin robotnik_controllers package (robotnik_base_controller) optional diff_drive_base_controller
Tested with mujoco 3.3.7 and ros 2 humble  

# INSTALL:

-Create mujoco_ws ros 2 workspace  
-Install mujoco  
-Install https://github.com/moveit/mujoco_ros2_control and compile  
-Install https://github.com/RobotnikAutomation/robotnik_mujoco and compile  

# DOCKER WITH ALL INSTALLED:

Create Image:

```
docker build -t docker/robotnik_mujoco_img .
```

Create Container:

```
sudo xhost +local:docker

docker run -it \
    --name mujoco_robotnik \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network host \
    --privileged \
    robotnik_mujoco_img
```

Open new Terminal in a running container:

```
docker exec -it mujoco_robotnik bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

# USAGE:

>ros2 launch robotnik_mujoco rbwatcher_world.launch.py


on a second terminal launch ps game pad or 

>ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robotnik_base_controller/cmd_vel_unstamped

for the diff drive controller:
>ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
