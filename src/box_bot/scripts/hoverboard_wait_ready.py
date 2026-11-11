#!/usr/bin/env python3
"""Gate controller startup on fresh, confirmed hub power telemetry."""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int8, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('hoverboard_wait_ready')
    node.declare_parameter('timeout_s', 15.0)
    ready = False
    description = False
    transforms = Buffer()
    listener = TransformListener(transforms, node)

    def receive(msg):
        nonlocal ready
        ready = msg.data == 1

    def model(msg):
        nonlocal description
        description = bool(msg.data.strip())

    node.create_subscription(String, '/robot_description', model,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # Volatile delivery requires a new hub heartbeat, avoiding cached on states.
    node.create_subscription(Int8, '/hoverboard/power_state', receive, 10)
    deadline = time.monotonic() + node.get_parameter('timeout_s').value
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if ready and description and transforms.can_transform('base_footprint', 'gps', Time()):
                node.get_logger().info('Hub confirms power on and robot model/GPS transforms are ready. Starting controller.')
                result = 0
                break
        if result:
            node.get_logger().error(f'Startup blocked. Power on={ready}, description={description}, '
                                    f'GPS transform={transforms.can_transform("base_footprint", "gps", Time())}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
