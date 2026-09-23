#!/usr/bin/env python3
"""Publish the combined control URDF with transient-local durability."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__("robot_description_publisher")
        self.declare_parameter("description_file", "")
        self.declare_parameter("topic_name", "robot_description")
        path = Path(str(self.get_parameter("description_file").value))
        if not path.is_file():
            raise RuntimeError(f"robot description file does not exist: {path}")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        topic_name = str(self.get_parameter("topic_name").value)
        self._publisher = self.create_publisher(String, topic_name, qos)
        self._message = String(data=path.read_text(encoding="utf-8"))
        self._publisher.publish(self._message)


def main() -> None:
    rclpy.init()
    node = RobotDescriptionPublisher()
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
