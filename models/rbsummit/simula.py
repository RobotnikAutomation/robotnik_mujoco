from ament_index_python.packages import get_package_share_directory

import time, math
import mujoco
import mujoco.viewer
import os

pkg_path = get_package_share_directory("robotnik_mujoco")
world_path = os.path.join(pkg_path, "worlds", "world.xml")
model = mujoco.MjModel.from_xml_path(world_path)

#model = mujoco.MjModel.from_xml_path("world.xml")
#model = mujoco.MjModel.from_xml_path("rbsummit.xml")
data = mujoco.MjData(model)

def qpos_index(joint_name: str) -> int:
    j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    return model.jnt_qposadr[j_id]

def qvel_index(joint_name: str) -> int:
    j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    return model.jnt_dofadr[j_id]


#base = qpos_index("base_free")
wheel_fl = qvel_index("robot_front_left_wheel_joint")
wheel_fr = qvel_index("robot_front_right_wheel_joint")   
wheel_bl = qvel_index("robot_back_left_wheel_joint")
wheel_br = qvel_index("robot_back_right_wheel_joint")

t0 = time.time()
with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running():
        t = time.time() - t0
        #angle = 2.0 * math.sin(0.5 * t)   # -2 .. 2 rad
        vel = 3.0

        # joints are continuous / hinge with no limits, any angle can be set
        #data.qpos[wheel_fl] = angle
        #data.qpos[wheel_fr] = angle
        data.qvel[wheel_fl] = -vel
        data.qvel[wheel_fr] = -vel
        data.qvel[wheel_bl] = -vel
        data.qvel[wheel_br] = -vel

        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
        v.sync()
