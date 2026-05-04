#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_publisher")
        self.declare_parameter("robot_description", "")
        self._description = self.get_parameter("robot_description").value
        if not self._description:
            raise RuntimeError("robot_description parameter must not be empty")

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._publisher = self.create_publisher(String, "robot_description", qos)
        self._timer = self.create_timer(5.0, self._publish)
        self._publish()

    def _publish(self):
        msg = String()
        msg.data = self._description
        self._publisher.publish(msg)


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
