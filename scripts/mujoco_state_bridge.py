#!/usr/bin/env python3
"""Publish per-robot odometry/TF and canonical-QoS IMU from MuJoCo state."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import TransformStamped
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


BASE_LINK_HEIGHT = 0.11


def _world_to_body(vector, orientation):
    """Rotate a world-frame vector into the robot frame."""
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vector[0]
        + 2.0 * (x * y + w * z) * vector[1]
        + 2.0 * (x * z - w * y) * vector[2],
        2.0 * (x * y - w * z) * vector[0]
        + (1.0 - 2.0 * (x * x + z * z)) * vector[1]
        + 2.0 * (y * z + w * x) * vector[2],
        2.0 * (x * z + w * y) * vector[0]
        + 2.0 * (y * z - w * x) * vector[1]
        + (1.0 - 2.0 * (x * x + y * y)) * vector[2],
    )


class MujocoStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_state_bridge")
        self.declare_parameter("num_robots", 1)
        self._count = int(self.get_parameter("num_robots").value)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._odom_publishers = {}
        static_transforms = []
        for index in range(1, self._count + 1):
            name = f"rbwatcher_{index}"
            self._odom_publishers[name] = self.create_publisher(Odometry, f"/{name}/odom", 10)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "world"
            transform.child_frame_id = f"{name}/odom"
            transform.transform.rotation.w = 1.0
            static_transforms.append(transform)
        self._static_tf.sendTransform(static_transforms)
        self.create_subscription(FreeJointStateArray, "/mujoco/free_joint_states", self._state_cb, 10)

    def _state_cb(self, msg: FreeJointStateArray) -> None:
        for state in msg.free_joints:
            suffix = "_base_link"
            if not state.name.endswith(suffix):
                continue
            robot = state.name[: -len(suffix)]
            publisher = self._odom_publishers.get(robot)
            if publisher is None:
                continue
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = f"{robot}/odom"
            odom.child_frame_id = f"{robot}/base_footprint"
            orientation = state.pose.pose.orientation

            # MuJoCo reports the free-joint pose at base_link.  ROS exposes the
            # mobile base pose at base_footprint, BASE_LINK_HEIGHT below it in
            # the robot's local Z direction.  Keeping that offset here makes
            # the dynamic and fixed halves of the TF chain compose exactly.
            local_z_world = (
                2.0 * (orientation.x * orientation.z + orientation.w * orientation.y),
                2.0 * (orientation.y * orientation.z - orientation.w * orientation.x),
                1.0 - 2.0 * (orientation.x**2 + orientation.y**2),
            )
            offset = tuple(-BASE_LINK_HEIGHT * value for value in local_z_world)
            odom.pose.pose.position.x = state.pose.pose.position.x + offset[0]
            odom.pose.pose.position.y = state.pose.pose.position.y + offset[1]
            odom.pose.pose.position.z = state.pose.pose.position.z + offset[2]
            odom.pose.pose.orientation = orientation

            angular_world = (
                state.twist.twist.angular.x,
                state.twist.twist.angular.y,
                state.twist.twist.angular.z,
            )
            linear_world = (
                state.twist.twist.linear.x,
                state.twist.twist.linear.y,
                state.twist.twist.linear.z,
            )
            # v_footprint = v_base_link + omega x r.  Odometry twists are
            # expressed in child_frame_id, whereas the MuJoCo message is world-frame.
            footprint_linear_world = (
                linear_world[0] + angular_world[1] * offset[2] - angular_world[2] * offset[1],
                linear_world[1] + angular_world[2] * offset[0] - angular_world[0] * offset[2],
                linear_world[2] + angular_world[0] * offset[1] - angular_world[1] * offset[0],
            )
            linear_body = _world_to_body(footprint_linear_world, orientation)
            angular_body = _world_to_body(angular_world, orientation)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = linear_body
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = angular_body
            publisher.publish(odom)

            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = state.pose.pose.position.x
            transform.transform.translation.y = state.pose.pose.position.y
            transform.transform.translation.z = state.pose.pose.position.z
            transform.transform.rotation = state.pose.pose.orientation
            self._tf.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = MujocoStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
