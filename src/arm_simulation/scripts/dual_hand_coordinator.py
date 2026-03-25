#!/usr/bin/env python3
"""
Dual Hand Coordinator

Subscribes to /finger_count (0-10) and splits the count across two hands:
  left  = min(count, 5)
  right = max(0, count - 5)

Examples:
  data: 8  → left=5, right=3
  data: 3  → left=3, right=0
  data: 10 → left=5, right=5
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class DualHandCoordinator(Node):

    def __init__(self):
        super().__init__('dual_hand_coordinator')

        self.left_pub  = self.create_publisher(Int32, '/left/finger_count',  10)
        self.right_pub = self.create_publisher(Int32, '/right/finger_count', 10)

        self.create_subscription(Int32, '/finger_count', self.finger_count_callback, 10)

        self.get_logger().info('Dual Hand Coordinator ready')
        self.get_logger().info('Send /finger_count (0-10) to control both hands')

    def finger_count_callback(self, msg):
        total = max(0, min(10, msg.data))

        left_count  = min(total, 5)
        right_count = max(0, total - 5)

        left_msg = Int32()
        left_msg.data = left_count
        self.left_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = right_count
        self.right_pub.publish(right_msg)

        self.get_logger().info(
            f'Total={total} → left={left_count}, right={right_count}')


def main(args=None):
    rclpy.init(args=args)
    node = DualHandCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
