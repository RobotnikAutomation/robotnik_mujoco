import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch_ros.actions import Node

import xacro


def generate_launch_description():	
    
    robotnik_mujoco_path = os.path.join(
        get_package_share_directory('robotnik_mujoco'))
        
    xacro_file = os.path.join(robotnik_mujoco_path,
                              'models/rbsummit',
                              'rbsummit.urdf')  # make sure the arg prefix:=robot_ when converting from xacro
                                  
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    #robotnik_controllers_path = os.path.join(
    #    get_package_share_directory('robotnik_controllers'))

    # TBD: use basic diff_drive_controller or the robotnik_controllers
    #controller_config_file = os.path.join(robotnik_controllers_path, 'config', 'rbsummit_controller_params.yaml')
    #controller_config_file = os.path.join(robotnik_mujoco_path, 'config', 'diff_drive_controller.yaml')
    controller_config_file = os.path.join(robotnik_mujoco_path, 'config', 'rbsummit_controller_params.yaml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path':os.path.join(robotnik_mujoco_path, 'worlds', 'world_nist_arenas.xml')}
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_robotnik_base_controller = ExecuteProcess(
        #cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        #     'diff_drive_base_controller'],
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robotnik_base_controller'],                  
        output='screen'
        #cmd=['ros2', 'topic', 'list'],
        #output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,                
                on_exit=[load_robotnik_base_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])
