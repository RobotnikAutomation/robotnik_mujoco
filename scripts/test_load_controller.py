#This script can be useful for debugging errors upon loading a controller in the controller_manager. Comment specific lines in the launch/rbsummit.launch.py for this test.

# Clean terminal
#source /opt/ros/humble/setup.bash
#source ~/mujoco_ws/install/setup.bash
#export MUJOCO_GL=egl             # (or osmesa if you don't have GPU/X11)
#export LD_LIBRARY_PATH=/opt/mujoco/bin:$LD_LIBRARY_PATH

# load controller_manager with the specific configuration
ros2 run controller_manager ros2_control_node \
  --ros-args \
  -p robot_description:="$(xacro /home/robert/mujoco_ws/src/robotnik_mujoco/models/rbsummit/rbsummit.urdf)" \
  --params-file /home/robert/mujoco_ws/src/robotnik_mujoco/config/diff_drive_controller.yaml

