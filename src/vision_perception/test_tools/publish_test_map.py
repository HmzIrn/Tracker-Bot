#!/usr/bin/env python3
"""Publish a small latched, all-free OccupancyGrid for goal-pose integration tests."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class TestMapPublisher(Node):
    def __init__(self):
        super().__init__("vision_perception_test_map")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos)
        self.message = OccupancyGrid()
        self.message.header.frame_id = "map"
        self.message.info.resolution = 0.1
        self.message.info.width = 100
        self.message.info.height = 100
        self.message.info.origin.position.x = -5.0
        self.message.info.origin.position.y = -5.0
        self.message.info.origin.orientation.w = 1.0
        self.message.data = [0] * (100 * 100)
        self.publisher.publish(self.message)
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = TestMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
